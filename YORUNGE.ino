#include <Servo.h>

// Servo motor tanımlamaları
Servo solkanat;
Servo sagkanat;
Servo kuyrukservo;
Servo motorgazservosu;
Servo ignitionservo;
Servo ignitionoffservo;

// PID ALGORİTMASI
double oransalkat = 2.0;   // Proportional Katsayı
double integralkat = 0.1;  // Integral Katsayı
double turevkat = 0.01;    // Derivative Katsayı

double integral = 0;
double turev = 0;
//ROLL PID
double roll = 1;
double hedefroll = 0;
double rollhata, rollsonhata = 0;
double rollturev = 0;
double rollintegral = 0;
double rollkontrolciktisi = 0;
//ROLL PID

double hedefburun = 10;  // Hedef burun açısı (pitch)
int burun = 11;          // Mevcut burun açısı (bu değeri sensörle güncellemelisiniz)
double hata, sonhata = 0;
double kontrolciktisi;

// Diğer değişkenler
bool motor = false;
bool gear = true;

void setup() {
  // Servo pinleri belirleniyor
  solkanat.attach(7);           // Sol kanat servosu
  sagkanat.attach(8);           // Sağ kanat servosu
  kuyrukservo.attach(2);        // Kuyruk servosu
  motorgazservosu.attach(12);   // Motor gaz servosu
  ignitionservo.attach(13);     // Motor ateşleme servosu
  ignitionoffservo.attach(14);  // Ateşlemeyi kapatan servo

  Serial.begin(9600);  // Seri iletişim başlatılıyor
}
void loop() {
  // Burun açısı kontrolü (cruisepitch fonksiyonu) burada çalışabilir
  pitchset();
  rollset();

  // Sürekli sensörlerden veri güncellenmeli, burun açısı sensör verisiyle ayarlanmalı
  // Örneğin bir IMU sensöründen burun açısını güncellemelisin
  // burun = getPitchFromSensor(); // Bu şekilde sensör verisini alabilirsin
}
void rollset() {
  rollhata = hedefroll - roll;
  rollintegral += rollhata;
  rollturev = rollhata - rollsonhata;
  rollkontrolciktisi = (oransalkat * rollhata) + (integralkat * rollintegral) + (turevkat * rollturev);
  int servoaciroll1 = constrain(90 + rollkontrolciktisi, 60, 120);
  int servoaciroll2 = constrain(90 - rollkontrolciktisi, 60, 120);

  solkanat.write(servoaciroll1);
  sagkanat.write(servoaciroll2);
  rollsonhata = rollhata;
}
// PID kontrol ile burun açısının ayarlanması
void pitchset() {
  hata = hedefburun - burun;  // Hata hesaplama
  integral += hata;           // Integral hesaplama
  turev = hata - sonhata;     // Derivative hesaplama
  kontrolciktisi = (oransalkat * hata) + (integralkat * integral) + (turevkat * turev);
  int servoaci = constrain(90 + kontrolciktisi, 60, 120);
  kuyrukservo.write(servoaci);
  sonhata = hata;  // hata güncellemesi
  delay(10);       // 10ms gecikme
}
// Servo motorların başlangıç kontrolü
void servocheck() {
  solkanat.write(90);
  sagkanat.write(90);
  kuyrukservo.write(90);
  motorgazservosu.write(90);
  delay(3000);

  solkanat.write(120);
  sagkanat.write(120);
  kuyrukservo.write(120);
  motorgazservosu.write(80);
  delay(3000);

  solkanat.write(90);
  sagkanat.write(90);
  kuyrukservo.write(90);
  motorgazservosu.write(90);
}
// Diğer sensör verilerinin kontrol edilmesi gerekir
void checklist() {
  if (motor == true && gear == true) {
    Serial.print("Checklist Tamamlandı. Hazır. Servolar Kontrol Moduna Geçiyor");
    servocheck();
  } else {
    Serial.print("Checklist Başarısız. Hazır Değil");
  }
}
// Motoru başlatan fonksiyon
void engineon() {
  ignitionservo.write(90);
  delay(1500);
  ignitionservo.write(180);
  motor = true;
}
// Motoru kapatan fonksiyon
void engineoff() {
  ignitionoffservo.write(90);
  delay(1000);
  ignitionoffservo.write(180);
  motor = false;
}
