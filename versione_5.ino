#include <arduinoFFT.h>

#define SAMPLES 64
#define SAMPLING_FREQUENCY 5000 // Hz
#define BUTTON_PIN 7
#define MODE_SWITCH 6
#define LED_QUARTO 8
#define LED_BATTUTA 9

const int analog_input = A0;
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// === BPM ===
const int beatBufferSize = 5;
unsigned long beatTimes[beatBufferSize];
int beatIndex = 0;
float bpmStimato = 0;
float bpmAttivo = 0;
bool bpmManuale = false;
unsigned long ultimoBeatMillis = 0;
unsigned long prossimoQuartoMillis = 0;
int quartoCorrente = 0;
unsigned long ultimoBeatRealeMillis = 0;

// === Auto-adattamento soglia ===
double sogliaEnergiaDinamica = 100.0;
const int mediaEnergiaFinestra = 50;
double energiaStorica[mediaEnergiaFinestra];
int energiaIndex = 0;

// === Pulsante ===
unsigned long ultimoClick = 0;
unsigned long clickTimes[4];
int clickCount = 0;

// === LED ===
bool ledOn = false;
unsigned long ledOffMillis = 0;

// === Modalità meter ===
bool peakOn = false;
unsigned long peakOffMillis = 0;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH, INPUT_PULLUP);
  pinMode(LED_QUARTO, OUTPUT);
  pinMode(LED_BATTUTA, OUTPUT);

  for (int i = 0; i < beatBufferSize; i++) beatTimes[i] = 0;
  for (int i = 0; i < mediaEnergiaFinestra; i++) energiaStorica[i] = sogliaEnergiaDinamica;

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), pulsantePremuto, FALLING);
}

void loop() {
  unsigned long ora = millis();
  bool modalitaBPM = digitalRead(MODE_SWITCH) == HIGH;

  // === CAMPIONAMENTO ===
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long t0 = micros();
    vReal[i] = analogRead(analog_input);
    vImag[i] = 0;
    while (micros() - t0 < (1000000 / SAMPLING_FREQUENCY));
  }

  // === FFT ===
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // === ENERGIA AUDIO (Kick esteso 40–300 Hz) ===
  double energiaAudio = 0;
  for (int i = 0; i < SAMPLES / 2; i++) {
    double freq = (i * SAMPLING_FREQUENCY) / SAMPLES;
    if (freq >= 40 && freq <= 300) {
      energiaAudio += vReal[i];
    }
  }

  // === Adattamento dinamico soglia ===
  energiaStorica[energiaIndex] = energiaAudio;
  energiaIndex = (energiaIndex + 1) % mediaEnergiaFinestra;
  double somma = 0;
  for (int i = 0; i < mediaEnergiaFinestra; i++) somma += energiaStorica[i];
  double energiaMedia = somma / mediaEnergiaFinestra;
  sogliaEnergiaDinamica = energiaMedia * 1.3;

  if (modalitaBPM) {
    // === Rilevamento beat per BPM ===
    if (energiaAudio > sogliaEnergiaDinamica) {
      if (ora - ultimoBeatMillis > 250) {
        beatTimes[beatIndex] = ora;
        beatIndex = (beatIndex + 1) % beatBufferSize;
        ultimoBeatMillis = ora;
        ultimoBeatRealeMillis = ora;

        float intervalli[beatBufferSize - 1];
        int validi = 0;
        for (int i = 1; i < beatBufferSize; i++) {
          int idx1 = (beatIndex + i - 1) % beatBufferSize;
          int idx0 = (beatIndex + i - 2) % beatBufferSize;
          if (beatTimes[idx1] > beatTimes[idx0]) {
            intervalli[validi++] = beatTimes[idx1] - beatTimes[idx0];
          }
        }

        if (validi >= 2) {
          float media = 0;
          for (int i = 0; i < validi; i++) media += intervalli[i];
          media /= validi;
          bpmStimato = 60000.0 / media;

          if (!bpmManuale) {
            if (abs(bpmStimato - bpmAttivo) > 10 || bpmAttivo == 0) {
              bpmAttivo = bpmStimato;
              prossimoQuartoMillis = ora;
              quartoCorrente = 0;
            }
          } else {
            if (abs(bpmStimato - bpmAttivo) > 20) {
              bpmManuale = false;
              bpmAttivo = bpmStimato;
              prossimoQuartoMillis = ora;
              quartoCorrente = 0;
            }
          }
        }
      }
    }

    // === QUARTI ===
    if (ora >= prossimoQuartoMillis && bpmAttivo > 0) {
      digitalWrite(LED_QUARTO, HIGH);
      ledOn = true;
      ledOffMillis = ora + 60;

      if (quartoCorrente == 0) {
        digitalWrite(LED_BATTUTA, HIGH);
      }

      prossimoQuartoMillis = ora + (60000.0 / bpmAttivo);
      quartoCorrente = (quartoCorrente + 1) % 4;
    }

    if (ledOn && ora >= ledOffMillis) {
      digitalWrite(LED_QUARTO, LOW);
      digitalWrite(LED_BATTUTA, LOW);
      ledOn = false;
    }

  } else {
    // === MODALITÀ METER ===
    // LED 8 (QUARTO) → VU meter
    int vuLevel = 0;
    if (energiaAudio > sogliaEnergiaDinamica * 2.0) vuLevel = 255;
    else if (energiaAudio > sogliaEnergiaDinamica * 1.5) vuLevel = 128;
    else if (energiaAudio > sogliaEnergiaDinamica * 1.1) vuLevel = 64;
    else vuLevel = 0;
    analogWrite(LED_QUARTO, vuLevel);

    // LED 9 (BATTUTA) → Peak meter
    if (energiaAudio > sogliaEnergiaDinamica * 2.5) {
      digitalWrite(LED_BATTUTA, HIGH);
      peakOn = true;
      peakOffMillis = ora + 100;
    }
    if (peakOn && ora >= peakOffMillis) {
      digitalWrite(LED_BATTUTA, LOW);
      peakOn = false;
    }
  }

  // === DEBUG ===
  Serial.print("MODE: "); Serial.print(modalitaBPM ? "BPM" : "METER");
  Serial.print(" | BPM ATTIVO: "); Serial.print(bpmAttivo, 1);
  Serial.print(" | Energia: "); Serial.print(energiaAudio, 1);
  Serial.print(" | Soglia: "); Serial.println(sogliaEnergiaDinamica, 1);

  delay(2);
}

// === PULSANTE SYNC BPM ===
void pulsantePremuto() {
  unsigned long ora = millis();
  if (ora - ultimoClick > 2000) clickCount = 0;
  ultimoClick = ora;
  clickTimes[clickCount] = ora;
  clickCount++;

  if (clickCount == 1) {
    // singolo click = sync battuta
    prossimoQuartoMillis = ora;
    quartoCorrente = 0;
  }

  if (clickCount == 4) {
    float intervalli[3];
    for (int i = 0; i < 3; i++) intervalli[i] = clickTimes[i + 1] - clickTimes[i];
    float media = (intervalli[0] + intervalli[1] + intervalli[2]) / 3.0;
    bpmAttivo = 60000.0 / media;
    bpmManuale = true;
    prossimoQuartoMillis = ora;
    quartoCorrente = 0;
    clickCount = 0;
  }
}
