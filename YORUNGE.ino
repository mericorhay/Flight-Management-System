#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// GPS için seri iletişim
int RX = 3;
int TX = 2;
int BAUD = 9600;
unsigned long oncezaman = 0;
const unsigned long gecikme = 3000;

SoftwareSerial gpsseri(RX, TX);  // RX 3 TX 2
TinyGPSPlus gps;

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

double max_integral = 30;  // Integral teriminin maksimum sınırı
double integral = 0;       // Integral terimi
double turev = 0;          // Türev terimi

// ROLL PID değişkenleri
double roll = 0;                   // Mevcut roll açısı
double hedefroll = 1;              // Hedef roll açısı
double rollhata, rollsonhata = 0;  // Roll hatası ve önceki hata
double rollturev = 0;              // Roll türevi
double rollintegral = 0;           // Roll integral
double rollkontrolciktisi = 0;     // Roll kontrol çıkışı

// PITCH PID değişkenleri
double hedefburun = 1.0;  // Hedef burun açısı (pitch)
double burun = 0;          // Mevcut burun açısı (sensörden güncellenmeli)
double hata, sonhata = 0;  // Hata ve önceki hata
double kontrolciktisi;     // Pitch kontrol çıkışı

int irtifa = 0;
// Diğer değişkenler
bool motor = false;  // Motor durumu
bool gear = true;    // İniş takımları durumu

// MPC kontrol parametreleri
double hedefas = 100.0;          // Hedef hava hızı
double mevcutas = 0.0;           // Mevcut hava hızı
double k = 1.0;                  // Kontrol kazancı
double deltat = 0.1;             // Zaman adımı
double durum[2] = { 0.0, 0.0 };  // [mevcut hız, kontrol girişi]
int numzamanadimi = 10;          // Zaman adımı sayısı
unsigned long prevmilis = 0;     // Önceki zaman kaydedici
long interval = 3000;            // Veri bildirim aralığı
//Mevcut Heading
int hedefhdg = 0;
int mevcuthdg = 0;
int hatahdg = 0;



void setup() {
  // Servo motor pinleri atanıyor
  solkanat.attach(7);          // Sol kanat servosu
  sagkanat.attach(8);          // Sağ kanat servosu
  kuyrukservo.attach(2);       // Kuyruk servosu
  motorgazservosu.attach(12);  // Motor gaz servosu

  ignitionservo.attach(13);     // Motor ateşleme servosu
  ignitionoffservo.attach(14);  // Ateşlemeyi kapatan servo
  gpsseri.begin(BAUD);
  

  Serial.begin(9600);  // Seri iletişim başlatılıyor
}

void loop() {
  verialimi();      // Veri alımını gerçekleştir
  MPC();            // MPC kontrol fonksiyonu çağrılıyor
  pitchset();       // Pitch kontrol fonksiyonu
  rollset();        // Roll kontrol fonksiyonu
  veribildirimi();  // Veri bildirim fonksiyonu
  delay(20);        // 70 ms gecikme
  getgps();         // GPS verisi alımı
}

void heading() {
  mevcuthdg = hdgal();
  hatahdg = hedefhdg - mevcuthdg;
  if (hatahdg < 5 && hatahdg > -5) {
    hedefroll = 0;
  } else {
    for (int i = 0; i < 7; i++) {
      hedefroll = i;
    }
  }
}



int hdgal() {
  return 90;
}

void takeoff() {
  hedefas = 60;
  hedefroll = 0;
  if (irtifa < 100) {
    gear = true;
  } else if (irtifa > 100) {
    gear = false;
  }
  if (mevcutas < 60) {
    hedefas = 120;
    delay(250);
    // TIRMANIŞ FONKSYİONUNA GEÇİŞ

    for (int i = 0; i < 8; i++) {
      hedefburun = i;
    }
    if (mevcutas < 50 && mevcutas > 60) {
      Serial.println("V1 Hızına Ulaşıldı Kalkış İptal Edilemez.");
    } else if (mevcutas < 60 && mevcutas > 70) {
      Serial.println("VR Hızına Ulaşıldı Burun Kaldırılıyor.");
    }
  }
}
void getgps() {
  unsigned long simdikizaman = millis();
  if (simdikizaman - oncezaman >= gecikme) {
    while (gpsseri.available() > 0) {
      gps.encode(gpsseri.read());
    }

    // GPS başlatma mesajını burada yazdır
    Serial.println("GPS Başlatıldı.");

    // Geçerli konum kontrolü
    if (gps.location.isValid()) {
      Serial.print("Enlem: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Boylam: ");
      Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("GPS konumu geçersiz.");
    }

    // Geçerli hız kontrolü
    if (gps.speed.isValid()) {
      Serial.print("Hız (km/saat): ");
      Serial.println(gps.speed.kmph());
    } else {
      Serial.println("GPS hızı geçersiz.");
    }

    // Geçerli yükseklik kontrolü
    if (gps.altitude.isValid()) {
      Serial.print("Yükseklik: ");
      Serial.println(gps.altitude.meters());
    } else {
      Serial.println("Yükseklik geçersiz.");
    }
    oncezaman = simdikizaman;
  }
}

// Hava hızı ölçüm fonksiyonu
double getairspeed() {
  return 95.0;  // Hava hızı verisi buradan sensörden alınır
}

void MPC() {
  mevcutas = getairspeed();  // Mevcut hava hızını al

  for (int i = 0; i < numzamanadimi; i++) {  // Zaman adımları döngüsü
    double mpchata = hedefas - mevcutas;     // Hedef ve mevcut hava hızı farkı

    durum[0] = mevcutas;                            // Mevcut hava hızını duruma kaydet
    durum[1] = k * mpchata;                         // Kontrol girişini hesapla
    int gazaci = constrain(90 + durum[1], 0, 180);  // Gaz servosu açısını sınırla

    mevcutas += durum[1] * deltat;  // Hız güncellemesi
    motorgazservosu.write(gazaci);  // Motor gaz servo açısını güncelle
  }
}

// Veri bildirim fonksiyonu
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

// Veri alım fonksiyonu
void verialimi() {
  if (Serial.available() > 0) {
    String gelenveri = Serial.readStringUntil('\n');  // Seri porttan gelen veriyi oku
    gelenveri.trim();
    if (gelenveri.startsWith("roll:")) {
      roll = gelenveri.substring(5).toDouble();  // Roll açısını güncelle
    }
    if (gelenveri.startsWith("reset:true")) {
      reset();
    }
    if (gelenveri.startsWith("burun:")) {
      burun = gelenveri.substring(6).toDouble();  // Burun açısını güncelle
    }
    if (gelenveri == "motor_on") {
      engineon();  // Motoru aç
    } else if (gelenveri == "motor_off") {
      engineoff();  // Motoru kapat
    }
  }
}

// Roll PID kontrolü ve servoları kilitleme
void rollset() {
  rollhata = hedefroll - roll;  // Roll hatasını hesapla

  // Eğer roll değeri hedefroll'e ulaştıysa, PID kontrolünü durdur
  if (abs(rollhata) <= 4) {  // 2 derecelik bir tolerans
    solkanat.write(90);      // Sol kanat servo açısını sabit tut
    sagkanat.write(90);      // Sağ kanat servo açısını sabit tut
    rollintegral = 0;        // Integral terimini sıfırla
    return;                  // Rollset fonksiyonunu burada sonlandır
  }

  // Integral terimini güncelle
  rollintegral += rollhata;

  // Integral teriminin sınırlandırılması
  if (rollintegral > max_integral) {
    rollintegral = max_integral;
  } else if (rollintegral < -max_integral) {
    rollintegral = -max_integral;
  }

  rollturev = rollhata - rollsonhata;  // Türev hesapla

  rollkontrolciktisi = (oransalkat * rollhata) + (integralkat * rollintegral) + (turevkat * rollturev);

  int servoaciroll1 = constrain(90 + rollkontrolciktisi, 60, 110);  // Sol kanat servo açısını sınırla
  int servoaciroll2 = constrain(90 - rollkontrolciktisi, 60, 110);  // Sağ kanat servo açısını sınırla

  solkanat.write(servoaciroll1);  // Sol kanat servo açısını güncelle
  sagkanat.write(servoaciroll2);  // Sağ kanat servo açısını güncelle

  rollsonhata = rollhata;  // Son hatayı güncelle
}

// Pitch PID kontrolü
void pitchset() {
  hata = hedefburun - burun;  // Hata hesapla

  // Eğer burun değeri hedefburun'a ulaştıysa, PID kontrolünü durdur
  if (abs(hata) <= 2) {   // 0.5 derecelik bir tolerans
    kuyrukservo.write(90);  // Kuyruk servo açısını sabit tut
    integral = 0;           // Integral terimini sıfırla
    return;                 // Pitchset fonksiyonunu burada sonlandır
  }

  integral += hata;  // Integral hesapla

  // Integral teriminin sınırlandırılması
  if (integral > max_integral) {
    integral = max_integral;
  } else if (integral < -max_integral) {
    integral = -max_integral;
  }

  turev = hata - sonhata;  // Türev hesapla

  kontrolciktisi = (oransalkat * hata) + (integralkat * integral) + (turevkat * turev);

  int servoaci = constrain(90 + kontrolciktisi, 60, 120);  // Servo açısını sınırla

  kuyrukservo.write(servoaci);  // Kuyruk servosu açısını güncelle

  sonhata = hata;  // Son hatayı güncelle
}

// Motor açma fonksiyonu
void engineon() {
  motorgazservosu.write(120);  // Gaz servosunu aç
  ignitionservo.write(120);    // Ateşleme servosunu aç
  motor = true;                // Motor durumunu aktif yap
  delay(3000);
}

// Motor kapama fonksiyonu
void engineoff() {
  motorgazservosu.write(60);    // Gaz servosunu kapat  //!!!!!!!!!!!!!!!!!!!!!!!
  ignitionoffservo.write(120);  // Ateşleme kapatma servosunu çalıştır //!!!!!!!!!!!!!!!!!!!!!!
  motor = false;                // Motor durumunu pasif yap
  delay(3000);
}

// Reset fonksiyonu
void reset() {
  hedefroll = 0;
  hedefburun = 0;
  rollintegral = 0;
  rollsonhata = 0;
  integral = 0;
  sonhata = 0;
  Serial.println("RESET Gerçekleşti");
}
