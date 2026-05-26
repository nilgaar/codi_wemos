#include <WiFi.h>
#include <LittleFS.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define AMPLADA_PANTALLA 128
#define ALCADA_PANTALLA 64
Adafruit_SSD1306 pantalla(AMPLADA_PANTALLA, ALCADA_PANTALLA, &Wire, -1);

struct __attribute__((packed)) FilaBD
{
  uint8_t bssid[6]; // 6 bytes binaris en lloc d'usar un String → estalvia uns 45 KB de DRAM
  int8_t rssi;
  float lat;
  float lon;
};

static FilaBD bd[3850]; // 3850 * 15 bytes = 57.75 KB
static int midaBDReal = 0;

// Converteix "aa:bb:cc:dd:ee:ff" (majúscules o minúscules) a 6 bytes binaris
static void parsejaBssid(const char *str, uint8_t *out)
{
  for (int i = 0; i < 6; i++)
    out[i] = (uint8_t)strtol(str + i * 3, nullptr, 16);
}

// Carregar CSV
// El CSV té format: bssid,rssi_dbm,latitude,longitude
// ha estat simplificat treient-li les columnes de freqüencia i precisió (accuracy).

static void parsejaLinia(const String &linia) // parseja i guarda
{
  if (midaBDReal >= 3850)
    return;

  int comes[3];
  int trobades = 0;
  for (int i = 0; i < (int)linia.length() && trobades < 3; i++)
  {
    if (linia[i] == ',')
      comes[trobades++] = i;
  }
  if (trobades < 3)
    return; // necessitem 3 comes per a 4 columnes

  String cadBssid = linia.substring(0, comes[0]);

  float lat = linia.substring(comes[1] + 1, comes[2]).toFloat();
  float lon = linia.substring(comes[2] + 1).toFloat();

  // Guardem la fila a la "BD"
  parsejaBssid(cadBssid.c_str(), bd[midaBDReal].bssid);
  bd[midaBDReal].rssi = (int8_t)linia.substring(comes[0] + 1, comes[1]).toInt();
  bd[midaBDReal].lat = lat;
  bd[midaBDReal].lon = lon;
  midaBDReal++;
}

static bool carregaCSV(const char *ruta)
{
  File fitxer = LittleFS.open(ruta, "r");
  if (!fitxer)
    return false;
  fitxer.readStringUntil('\n'); // salta la capçalera
  while (fitxer.available())    // available retorna si queden bytes per llegir
  {
    String linia = fitxer.readStringUntil('\n');
    linia.trim();
    parsejaLinia(linia);
  }
  fitxer.close();
  return true;
}

static void mostraMissatge(const char *l1, const char *l2 = nullptr, const char *l3 = nullptr) // per la pantalla oled
{
  pantalla.clearDisplay();
  pantalla.setTextSize(1);
  pantalla.setTextColor(SSD1306_WHITE);
  pantalla.setCursor(0, 0);
  pantalla.println(l1);
  if (l2)
    pantalla.println(l2);
  if (l3)
    pantalla.println(l3);
  pantalla.display();
}

// Algoritme 1-NN
// Per a cada fila de la BD, comprova si el BSSID corresponent es troba en l'escaneix d'aquest cicle.
// Si coincideix, calcula (rssi_mesurat - rssi_bd)^2 i guarda el mínim.
// La posició de la fila amb menor distància és l'estimació.
static bool estimaPosicio(float &latSortida, float &lonSortida, int &coincidenciesSortida)
{
  int nXarxes = WiFi.scanNetworks();
  if (nXarxes < 1)
    return false;

  // Convertim els BSSIDs escanejats a bytes una vegada per evitar conversions repetides
  const int NX = min(nXarxes, 40);
  uint8_t bssidEscaneig[40][6];
  int rssiEscaneig[40];
  for (int j = 0; j < NX; j++)
  {
    parsejaBssid(WiFi.BSSIDstr(j).c_str(), bssidEscaneig[j]);
    rssiEscaneig[j] = WiFi.RSSI(j);
  }

  float millorDist = 1e9f; // inicialitzem a un valor exageradament gran
  int millorIdx = -1;
  coincidenciesSortida = 0;

  for (int i = 0; i < midaBDReal; i++)
  {
    for (int j = 0; j < NX; j++)
    {
      if (memcmp(bssidEscaneig[j], bd[i].bssid, 6) == 0)
      {
        float dif = (float)rssiEscaneig[j] - (float)bd[i].rssi;
        float distancia = dif * dif;
        if (distancia < millorDist)
        {
          millorDist = distancia;
          millorIdx = i;
        }
        coincidenciesSortida++;
        break;
      }
    }
  }

  if (millorIdx < 0)
    return false;
  latSortida = bd[millorIdx].lat;
  lonSortida = bd[millorIdx].lon;
  return true;
}

void setup()
{
  Serial.begin(115200); // velocitat de comunicació per monitoritzar per USB.
  // No afecta a la velocitat de l'escaneig WiFi ni a l'estimació de posició, però no definir-la provoca problemes en traspassar l'escript a la placa

  Wire.begin(21, 22); // SDA=21, SCL=22 (Wemos D1 R32). Paràmetres per la pantalla OLED.
  if (!pantalla.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED no trobada");
    while (true)
      delay(1000);
  }
  mostraMissatge("Iniciant...");

  if (!LittleFS.begin(true))
  {
    mostraMissatge("Error LittleFS");
    while (true)
      delay(1000);
  }

  mostraMissatge("Carregant CSV...");
  if (!carregaCSV("/wardrive.csv"))
  {
    mostraMissatge("Fitxer no trobat:", "/wardrive.csv");
    while (true)
      delay(1000);
  }

  char text[32];
  snprintf(text, sizeof(text), "Carregat: %d files", midaBDReal);
  mostraMissatge(text, "BD llesta.");
  Serial.printf("DB carregada: %d files\n", midaBDReal);
  delay(2000); // mostrem el missatge uns segons, altrament no es pot llegir

  // Per tal de poder adjuntar un timestamp a les estimacions, necessitem la placa tingui un valor de temps inicial
  // La font més accessible és el Wi-Fi Hotspot del telèfon mòbil.
  // El temps es sincronitza mitjançant NTP (Network Time Protocol)
  // Es connecta només una vegada. Sense aquest pas la placa no s'inicia
  mostraMissatge("Connectant WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin("moto", "");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  mostraMissatge("Sincronitzant NTP...");
  configTime(7200, 0, "pool.ntp.org"); // UTC+2 (CEST)
  time_t ara = 0;
  while (ara < 100000)
  {
    delay(200);
    ara = time(nullptr);
  } // espera hora vàlida
  Serial.println("Hora sincronitzada.");

  WiFi.disconnect(true);
  delay(100);
}

// ── Loop ─────────────────────────────────────────────────────────────────
void loop()
{
  mostraMissatge("Escanejant...");

  float lat = 0, lon = 0;
  int coincidencies = 0;

  if (estimaPosicio(lat, lon, coincidencies))
  {
    time_t ara = time(nullptr);
    struct tm *t = localtime(&ara);
    char horaText[12];
    strftime(horaText, sizeof(horaText), "%H:%M:%S", t);

    char linia1[24], linia2[24], linia3[24];
    snprintf(linia1, sizeof(linia1), "Lat: %.5f", lat);
    snprintf(linia2, sizeof(linia2), "Lon: %.5f", lon);
    snprintf(linia3, sizeof(linia3), "Coincid: %d", coincidencies);

    pantalla.clearDisplay();
    pantalla.setTextSize(1);
    pantalla.setTextColor(SSD1306_WHITE);
    pantalla.setCursor(0, 0);
    pantalla.println(horaText);
    pantalla.println(linia1);
    pantalla.println(linia2);
    pantalla.println(linia3);
    pantalla.display();

    Serial.printf("Estimat: %.5f, %.5f  (coincidencies=%d)\n", lat, lon, coincidencies);

    File log = LittleFS.open("/log.csv", "a");
    if (log)
    {
      char timestamp[24];
      strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", t);
      log.printf("%s,%.5f,%.5f\n", timestamp, lat, lon);
      log.close();
    }
  }
  else
  {
    mostraMissatge("Sense coincid.", "Reintentant...");
    Serial.println("Sense coincidencies");
  }

  delay(5000); // espera uns segons abans de tornar a escanejar
}
