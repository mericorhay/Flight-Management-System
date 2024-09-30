#include <Servo.h>

// Servo motor tanımlamaları
Servo solkanat;
Servo sagkanat;
Servo kuyrukservo;
Servo motorgazservosu;
Servo ignitionservo;
Servo ignitionoffservo;

// PID kontrol parametreleri
double oransalkat = 1.0;   // Proportional Katsayı
double integralkat = 0.3;  // Integral Katsayı
double turevkat = 0.02;    // Derivative Katsayı

double max_integral = 100;  // Integral teriminin maksimum sınırı
double integral = 0;        // Integral terimi
double turev = 0;           // Türev terimi

// ROLL PID değişkenleri
double roll = 0;                 // Mevcut roll açısı
double hedefroll = 20;          // Hedef roll açısı simülasyonu için
double rollhata, rollsonhata = 0; // Roll hatası ve önceki hata
double rollturev = 0;            // Roll türevi
double rollintegral = 0;         // Roll integral
double rollkontrolciktisi = 0;   // Roll kontrol çıkışı

// PITCH PID değişkenleri
double hedefburun = 1;  // Hedef burun açısı (pitch)
double burun = 0;       // Mevcut burun açısı (sensörden güncellenmeli)
double hata, sonhata = 0; // Hata ve önceki hata
double kontrolciktisi;  // Pitch kontrol çıkışı

// Diğer değişkenler
bool motor = false;     // Motor durumu
bool gear = true;       // İniş takımları durumu

// MPC kontrol parametreleri
double hedefas = 100.0;  // Hedef hava hızı
double mevcutas = 0.0;   // Mevcut hava hızı
double k = 1.0;          // Kontrol kazancı
double deltat = 0.1;     // Zaman adımı
double durum[2] = { 0.0, 0.0 }; // [mevcut hız, kontrol girişi]
int numzamanadimi = 10;  // Zaman adımı sayısı
unsigned long prevmilis = 0; // Önceki zaman kaydedici
long interval = 3000;    // Veri bildirim aralığı

void setup() {
  // Servo motor pinleri atanıyor
  solkanat.attach(7);          // Sol kanat servosu
  sagkanat.attach(8);          // Sağ kanat servosu
  kuyrukservo.attach(2);       // Kuyruk servosu
  motorgazservosu.attach(12);  // Motor gaz servosu

  ignitionservo.attach(13);     // Motor ateşleme servosu
  ignitionoffservo.attach(14);  // Ateşlemeyi kapatan servo

  Serial.begin(9600);  // Seri iletişim başlatılıyor
}

void loop() {
  verialimi();          // Veri alımını gerçekleştir
  MPC();                // MPC kontrol fonksiyonu çağrılıyor
  pitchset();          // Pitch kontrol fonksiyonu
  rollset();           // Roll kontrol fonksiyonu
  veribildirimi();     // Veri bildirim fonksiyonu
  delay(3000);         // Kısa bir gecikme
  roll = roll + 1;     // Roll açısı simülasyonu için +1 artış
}

// Hava hızı ölçüm fonksiyonu
double getairspeed(){
  return 95.0; // Hava hızı verisi buradan sensörden alınır
}

void MPC() {
  mevcutas = getairspeed(); // Mevcut hava hızını al

  for (int i = 0; i < numzamanadimi; i++) { // Zaman adımları döngüsü
    double mpchata = hedefas - mevcutas; // Hedef ve mevcut hava hızı farkı

    durum[0] = mevcutas;                  // Mevcut hava hızını duruma kaydet
    durum[1] = k * mpchata;               // Kontrol girişini hesapla
    int gazaci = constrain(90 + durum[1], 0, 180); // Gaz servosu açısını sınırla

    mevcutas += durum[1] * deltat; // Hız güncellemesi
    motorgazservosu.write(gazaci);  // Motor gaz servo açısını güncelle
  }
}

void veribildirimi() {
  unsigned long currentmilis = millis();
  if (currentmilis - prevmilis >= interval) {
    prevmilis = currentmilis;

    // Bilgileri seri port üzerinden gönder
    Serial.println("----------------------------");
    Serial.println(String("Hedef Hava Hızı: ") + hedefas);
    Serial.println(String("Mevcut Hava Hızı: ") + mevcutas);
    Serial.println(String("Motor Kontrol Servosu Durumu: ") + motorgazservosu.read());
    Serial.println(String("Hedef Roll Açısı: ") + hedefroll);
    Serial.println(String("Hedef Burun Açısı: ") + hedefburun);
    Serial.println(String("Anlık Burun Açısı: ") + burun);
    Serial.println(String("Anlık Roll Açısı: ") + roll);
    Serial.println(String("İniş Takımları Durumu: ") + gear);
    Serial.println(String("Motor Durumu: ") + motor);
    Serial.println(String("Roll Kontrol Çıkışı: ") + rollkontrolciktisi);
    Serial.println("----------------------------");
  }
}

void verialimi() {
  if (Serial.available() > 0) {
    String gelenveri = Serial.readStringUntil('\n'); // Seri porttan gelen veriyi oku
    gelenveri.trim();
    if (gelenveri.startsWith("roll:")) {
      hedefroll = gelenveri.substring(5).toDouble(); // Roll açısını güncelle
    }
    if (gelenveri.startsWith("burun:")) {
      hedefburun = gelenveri.substring(6).toDouble(); // Burun açısını güncelle
    }
    if (gelenveri == "motor_on") {
      engineon(); // Motoru aç
    } else if (gelenveri == "motor_off") {
      engineoff(); // Motoru kapat
    }
  }
}

// Roll PID kontrolü ve servoları kilitleme
void rollset() {
  rollhata = hedefroll - roll; // Roll hatasını hesapla

  // Eğer roll değeri hedefroll'e ulaştıysa, PID kontrolünü durdur
  if (abs(rollhata) <= 0.5) {  // 0.5 derecelik bir tolerans
    // Servoları mevcut açıda tutarak kilitleme durumu
    solkanat.write(90);  // Sol kanat servo açısını sabit tut
    sagkanat.write(90);  // Sağ kanat servo açısını sabit tut
    return;  // Rollset fonksiyonunu burada sonlandır
  }

  rollintegral += rollhata;

  // Integral teriminin sınırlandırılması
  if (rollintegral > max_integral) {
    rollintegral = max_integral;
  } else if (rollintegral < -max_integral) {
    rollintegral = -max_integral;
  }

  rollturev = rollhata - rollsonhata; // Türev hesapla

  rollkontrolciktisi = (oransalkat * rollhata) + (integralkat * rollintegral) + (turevkat * rollturev);

  int servoaciroll1 = constrain(90 + rollkontrolciktisi, 60, 120); // Sol kanat servo açısını sınırla
  int servoaciroll2 = constrain(90 - rollkontrolciktisi, 60, 120); // Sağ kanat servo açısını sınırla

  solkanat.write(servoaciroll1); // Sol kanat servo açısını güncelle
  sagkanat.write(servoaciroll2);  // Sağ kanat servo açısını güncelle
  rollsonhata = rollhata;          // Son hatayı güncelle
}

// Pitch PID kontrolü ve servoları kilitleme
void pitchset() {
  hata = hedefburun - burun; // Pitch hatasını hesapla

  // Eğer burun değeri hedefburun'a ulaştıysa, PID kontrolünü durdur
  if (abs(hata) <= 0.5) {  // 0.5 derecelik bir tolerans
    // Kuyruk servosunu mevcut açıda tutarak kilitleme durumu
    kuyrukservo.write(90);  // Kuyruk servo açısını sabit tut
    return;  // Pitchset fonksiyonunu burada sonlandır
  }

  integral += hata;

  // Integral teriminin sınırlandırılması
  if (integral > max_integral) {
    integral = max_integral;
  } else if (integral < -max_integral) {
    integral = -max_integral;
  }

  turev = hata - sonhata; // Türev hesapla
  kontrolciktisi = (oransalkat * hata) + (integralkat * integral) + (turevkat * turev);

  int servoacipitch = constrain(90 + kontrolciktisi, 60, 120); // Kuyruk servo açısını sınırla
  kuyrukservo.write(servoacipitch); // Kuyruk servo açısını güncelle
  sonhata = hata; // Son hatayı güncelle
}

// Motoru açan fonksiyon
void engineon() {
  motor = true; // Motor durumunu aç
  ignitionservo.write(180); // Ateşleme servosunu aç
  ignitionoffservo.write(0); // Motoru aç
  Serial.println("Motor açıldı");
}

// Motoru kapatan fonksiyon
void engineoff() {
  motor = false; // Motor durumunu kapat
  ignitionoffservo.write(90); // Motoru kapat
  ignitionservo.write(0); // Ateşleme servosunu kapat
  Serial.println("Motor kapatıldı");
}
#include <Servo.h>

// Servo motor tanımlamaları
Servo solkanat;
Servo sagkanat;
Servo kuyrukservo;
Servo motorgazservosu;
Servo ignitionservo;
Servo ignitionoffservo;

// PID kontrol parametreleri
double oransalkat = 1.0;   // Proportional Katsayı
double integralkat = 0.3;  // Integral Katsayı
double turevkat = 0.02;    // Derivative Katsayı

double max_integral = 100;  // Integral teriminin maksimum sınırı
double integral = 0;        // Integral terimi
double turev = 0;           // Türev terimi

// ROLL PID değişkenleri
double roll = 0;                 // Mevcut roll açısı
double hedefroll = 20;          // Hedef roll açısı simülasyonu için
double rollhata, rollsonhata = 0; // Roll hatası ve önceki hata
double rollturev = 0;            // Roll türevi
double rollintegral = 0;         // Roll integral
double rollkontrolciktisi = 0;   // Roll kontrol çıkışı

// PITCH PID değişkenleri
double hedefburun = 1;  // Hedef burun açısı (pitch)
double burun = 0;       // Mevcut burun açısı (sensörden güncellenmeli)
double hata, sonhata = 0; // Hata ve önceki hata
double kontrolciktisi;  // Pitch kontrol çıkışı

// Diğer değişkenler
bool motor = false;     // Motor durumu
bool gear = true;       // İniş takımları durumu

// MPC kontrol parametreleri
double hedefas = 100.0;  // Hedef hava hızı
double mevcutas = 0.0;   // Mevcut hava hızı
double k = 1.0;          // Kontrol kazancı
double deltat = 0.1;     // Zaman adımı
double durum[2] = { 0.0, 0.0 }; // [mevcut hız, kontrol girişi]
int numzamanadimi = 10;  // Zaman adımı sayısı
unsigned long prevmilis = 0; // Önceki zaman kaydedici
long interval = 3000;    // Veri bildirim aralığı

void setup() {
  // Servo motor pinleri atanıyor
  solkanat.attach(7);          // Sol kanat servosu
  sagkanat.attach(8);          // Sağ kanat servosu
  kuyrukservo.attach(2);       // Kuyruk servosu
  motorgazservosu.attach(12);  // Motor gaz servosu

  ignitionservo.attach(13);     // Motor ateşleme servosu
  ignitionoffservo.attach(14);  // Ateşlemeyi kapatan servo

  Serial.begin(9600);  // Seri iletişim başlatılıyor
}

void loop() {
  verialimi();          // Veri alımını gerçekleştir
  MPC();                // MPC kontrol fonksiyonu çağrılıyor
  pitchset();          // Pitch kontrol fonksiyonu
  rollset();           // Roll kontrol fonksiyonu
  veribildirimi();     // Veri bildirim fonksiyonu
  delay(3000);         // Kısa bir gecikme
  roll = roll + 1;     // Roll açısı simülasyonu için +1 artış
}

// Hava hızı ölçüm fonksiyonu
double getairspeed(){
  return 95.0; // Hava hızı verisi buradan sensörden alınır
}

void MPC() {
  mevcutas = getairspeed(); // Mevcut hava hızını al

  for (int i = 0; i < numzamanadimi; i++) { // Zaman adımları döngüsü
    double mpchata = hedefas - mevcutas; // Hedef ve mevcut hava hızı farkı

    durum[0] = mevcutas;                  // Mevcut hava hızını duruma kaydet
    durum[1] = k * mpchata;               // Kontrol girişini hesapla
    int gazaci = constrain(90 + durum[1], 0, 180); // Gaz servosu açısını sınırla

    mevcutas += durum[1] * deltat; // Hız güncellemesi
    motorgazservosu.write(gazaci);  // Motor gaz servo açısını güncelle
  }
}

void veribildirimi() {
  unsigned long currentmilis = millis();
  if (currentmilis - prevmilis >= interval) {
    prevmilis = currentmilis;

    // Bilgileri seri port üzerinden gönder
    Serial.println("----------------------------");
    Serial.println(String("Hedef Hava Hızı: ") + hedefas);
    Serial.println(String("Mevcut Hava Hızı: ") + mevcutas);
    Serial.println(String("Motor Kontrol Servosu Durumu: ") + motorgazservosu.read());
    Serial.println(String("Hedef Roll Açısı: ") + hedefroll);
    Serial.println(String("Hedef Burun Açısı: ") + hedefburun);
    Serial.println(String("Anlık Burun Açısı: ") + burun);
    Serial.println(String("Anlık Roll Açısı: ") + roll);
    Serial.println(String("İniş Takımları Durumu: ") + gear);
    Serial.println(String("Motor Durumu: ") + motor);
    Serial.println(String("Roll Kontrol Çıkışı: ") + rollkontrolciktisi);
    Serial.println("----------------------------");
  }
}

void verialimi() {
  if (Serial.available() > 0) {
    String gelenveri = Serial.readStringUntil('\n'); // Seri porttan gelen veriyi oku
    gelenveri.trim();
    if (gelenveri.startsWith("roll:")) {
      hedefroll = gelenveri.substring(5).toDouble(); // Roll açısını güncelle
    }
    if (gelenveri.startsWith("burun:")) {
      hedefburun = gelenveri.substring(6).toDouble(); // Burun açısını güncelle
    }
    if (gelenveri == "motor_on") {
      engineon(); // Motoru aç
    } else if (gelenveri == "motor_off") {
      engineoff(); // Motoru kapat
    }
  }
}

// Roll PID kontrolü ve servoları kilitleme
void rollset() {
  rollhata = hedefroll - roll; // Roll hatasını hesapla

  // Eğer roll değeri hedefroll'e ulaştıysa, PID kontrolünü durdur
  if (abs(rollhata) <= 0.5) {  // 0.5 derecelik bir tolerans
    // Servoları mevcut açıda tutarak kilitleme durumu
    solkanat.write(90);  // Sol kanat servo açısını sabit tut
    sagkanat.write(90);  // Sağ kanat servo açısını sabit tut
    return;  // Rollset fonksiyonunu burada sonlandır
  }

  rollintegral += rollhata;

  // Integral teriminin sınırlandırılması
  if (rollintegral > max_integral) {
    rollintegral = max_integral;
  } else if (rollintegral < -max_integral) {
    rollintegral = -max_integral;
  }

  rollturev = rollhata - rollsonhata; // Türev hesapla

  rollkontrolciktisi = (oransalkat * rollhata) + (integralkat * rollintegral) + (turevkat * rollturev);

  int servoaciroll1 = constrain(90 + rollkontrolciktisi, 60, 120); // Sol kanat servo açısını sınırla
  int servoaciroll2 = constrain(90 - rollkontrolciktisi, 60, 120); // Sağ kanat servo açısını sınırla

  solkanat.write(servoaciroll1); // Sol kanat servo açısını güncelle
  sagkanat.write(servoaciroll2);  // Sağ kanat servo açısını güncelle
  rollsonhata = rollhata;          // Son hatayı güncelle
}

// Pitch PID kontrolü ve servoları kilitleme
void pitchset() {
  hata = hedefburun - burun; // Pitch hatasını hesapla

  // Eğer burun değeri hedefburun'a ulaştıysa, PID kontrolünü durdur
  if (abs(hata) <= 0.5) {  // 0.5 derecelik bir tolerans
    // Kuyruk servosunu mevcut açıda tutarak kilitleme durumu
    kuyrukservo.write(90);  // Kuyruk servo açısını sabit tut
    return;  // Pitchset fonksiyonunu burada sonlandır
  }

  integral += hata;

  // Integral teriminin sınırlandırılması
  if (integral > max_integral) {
    integral = max_integral;
  } else if (integral < -max_integral) {
    integral = -max_integral;
  }

  turev = hata - sonhata; // Türev hesapla
  kontrolciktisi = (oransalkat * hata) + (integralkat * integral) + (turevkat * turev);

  int servoacipitch = constrain(90 + kontrolciktisi, 60, 120); // Kuyruk servo açısını sınırla
  kuyrukservo.write(servoacipitch); // Kuyruk servo açısını güncelle
  sonhata = hata; // Son hatayı güncelle
}

// Motoru açan fonksiyon
void engineon() {
  motor = true; // Motor durumunu aç
  ignitionservo.write(180); // Ateşleme servosunu aç
  ignitionoffservo.write(0); // Motoru aç
  Serial.println("Motor açıldı");
}

// Motoru kapatan fonksiyon
void engineoff() {
  motor = false; // Motor durumunu kapat
  ignitionoffservo.write(90); // Motoru kapat
  ignitionservo.write(0); // Ateşleme servosunu kapat
  Serial.println("Motor kapatıldı");
}
