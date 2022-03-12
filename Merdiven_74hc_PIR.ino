/********************************************************************
   ŞUBAT 2022 Dr.TRonik YouTube
   Nano + 8x74HC595 ile 1-64 basamak ayarlanabilir sıralı aydınlatma
   2 adet PIR sensör HC SR501/505
   Max. 64 Kanal Mosfet veya Röle Kartı
   Basamak sayısı enfazla 64 arası ve ayarlanabilir...
   Basamak yanma ve sönme süreleri ile bekleme süresi trimpot ile ayarlanabilir...
 ********************************************************************/

/********************************************************************
  GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___
 ********************************************************************/
//Kütüphaneler...
#include <LiquidCrystal_I2C.h> //I²C li LCD için gerekli...
#include <Wire.h>      //I²C için gerekli...
#include <RTClib.h>   //Saat modulu için gerekli...

//Nesne tanımları...
RTC_DS3231 rtc; //Saat modulü ve verilen nesne ismi tanımı...
LiquidCrystal_I2C lcd(0x3F, 20, 4);  //Kullandığımız LCD I²C adres ve karakter satır sayısı tanımlama

//Değişken tanımları...
int dataPin = 8;    //DS -> 14.pin
int latchPin = 9;   //ST_CP -> 12.Pin
int clockPin = 10;  // SH_CP -> 11.Pin

boolean registers[65]; //ShiftRegister entegrelerimizde kayıt açma...

int wt_okunan ; //Tüm basamaklar yandıktan sonra geri sönme süresi potansiyometreden okunan
int st_okunan ; // Basamakların yanma sönme süresi potansiyometreden okunan
int wt_; //waiting time
int st_; //stairs_time
int bs_okunan; //bs_ pot değeri
int ldr_; //LDR den okunan Işık şiddeti için...
int stairs_count; //Basamak sayısı

#define sens_1 3 //1.Sensor  D3 nolu pin
#define sens_2 4 //2.Sensor  D4 nolu pin

boolean sens_1_deger , sens_2_deger = HIGH;

boolean ilk_calisma = true ;

int output = 12; //Kart çıkışının high/mosfet ya da low/relay olarak ayarlanması
boolean card_out ;
int tam_yan = 11; //Tüm kanalların yanması için
boolean flag = true ;

//LCD için ekran lamba görüntüsü...
byte lamp[8] = {
  B11111,   //-----
  B00000,   //
  B11111,   //-----
  B00000,   //
  B11111,   //-----
  B01110,   // ---
  B01110,   // ---
  B00100    //  -
};



/********************************************************************
  SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___
 ********************************************************************/
void setup() {
  Serial.begin (9600); //Bluetooth modülüyle iletişim için gerekli...

  //Pin mod tanımları...
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  pinMode(sens_1, INPUT);
  pinMode(sens_2, INPUT);

  pinMode(output, INPUT_PULLUP);
  pinMode(tam_yan, INPUT_PULLUP);

  //İlk çalıştırma...
  for (int i = 0; i < 65; i++) {
    registers[i] = HIGH;
    writereg();
  }

  //Kütüphane nesneleri başlama komutları...
  lcd.begin();  //LCD başlasın
  lcd.clear();  // ve ekran temizlensin
  lcd.backlight(); //lcd arka plan yansın
  lcd.createChar(0, lamp); //lamp veri tipinden 0. karekter oluşturulsun
  rtc.begin(); //Saat modulu başlasın...

  // Yazılım ile saati ayarlama, aşağıdaki satırı aktifleştirip yükledikten sonra satırı pasifleştirip kodu tekrar yüklemek gerekli!
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

/********************************************************************
  LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__
 ********************************************************************/
void loop() {
  sens_1_deger = digitalRead(sens_1);
  sens_2_deger = digitalRead(sens_2);
  //Tek defa çalışacak komutlar if bloğunun içinde...
  if (ilk_calisma == true)
  {
    all_on();
    if (sens_1_deger == HIGH && sens_2_deger == HIGH)
    {
      return;
    }
    else {
      all_off();
    }
  }
  ilk_calisma = false;

  //Sıralı aydınlatma komutları...


  if (sens_1_deger == HIGH) {
    sens_u();
  }

  if (sens_2_deger == HIGH) {
    sens_d();
  }

  //Tüm basamaklar yandıktan sonra geri sönme gecikme süresini 1 saniye ile 5 saniye arasında ayralamak için
  //A0 dan okunan analog pot giriş seviyesini  süreye aktarma...
  wt_okunan = analogRead(A0);
  if (wt_okunan > 0 && wt_okunan < 204) {
    wt_ = 1000;
  }
  else if (wt_okunan > 203 && wt_okunan < 408) {
    wt_ = 2000;
  }
  else if (wt_okunan > 407 && wt_okunan < 612) {
    wt_ = 3000;
  }
  else if (wt_okunan > 611 && wt_okunan < 816) {
    wt_ = 4000;
  }
  else if (wt_okunan > 815 && wt_okunan < 1024) {
    wt_ = 5000;
  }

  //Her bir basamak yanma sönme sürelerini ~0,2 saniye ile ~1 saniye arasında ayralamak için
  //A1 den okunan analog pot giriş seviyesini  süreye aktarma...
  st_okunan = analogRead(A1);
  if (st_okunan < 200) {
    st_ = 200;
  }
  else {
    (st_ = st_okunan);
  }

  //Basamak sayısını; A2 den okunan analog değere göre 1 - 64 arası mapping yapılarak atama...
  //bs_okunan = analogRead(A2);

  stairs_count = 64;

  //LDR den alınan ışık şiddeti bilgisini A3 den direkt alalım ve 1-8 arasına eşleştirelim...
  ldr_ = map(analogRead(A3), 0, 1023, 1, 8);

  //Saat ile ilgili komutlar...
  DateTime now = rtc.now();
  char saat[] = "hh:mm";
  //char tarih[] = "DD/MM/YY";

  //LCD ile ilgili komutlar...
  //ORtam aydınlıksa lcd arka ışık yanmasın
  if (ldr_ > 6) {
    lcd.noBacklight();
  }
  else {
    lcd.backlight();
  }
  lcd.setCursor(0, 0);
  lcd.print("BASAMAK SAYISI:");
  lcd.setCursor(16, 0);
  lcd.print(stairs_count);
  lcd.setCursor(0, 1);
  lcd.print("Yan-Son:");
  lcd.setCursor(9, 1);
  lcd.print((float)st_ / 1000, 1) + lcd.print("sn.");
  lcd.setCursor(17, 1);
  lcd.write(0);
  lcd.setCursor(19, 1);
  lcd.print(ldr_);
  lcd.setCursor(0, 2);
  lcd.print("Bekleme:");
  lcd.setCursor(9, 2);
  lcd.print(wt_ / 1000) + lcd.print("sn.");
  lcd.setCursor(16, 2);
  lcd.print((float)(rtc.getTemperature() - 1), 0) +  lcd.print("'C");
  lcd.setCursor(0, 3);
  //lcd.print(now.toString(tarih));
  lcd.setCursor(15, 3);
  lcd.print(now.toString(saat));

  //Kart çıkışını nosfet veya röle kartına göre ayarlama...
  output = digitalRead(12);
  if (output == LOW) {
    lcd.setCursor(0, 3);
    lcd.print("MOSFET /");
    card_out = HIGH;
  }
  if (output == HIGH) {
    lcd.setCursor(0, 3);
    lcd.print("ROLE   /");
    card_out = LOW;
  }


  //

  //Tüm ayarlanan aydınlatmaların yanık kalması
  //  tam_yan = digitalRead(11);
  //  if (tam_yan == LOW) {
  //    lcd.setCursor(9, 3);
  //    lcd.print("TST /");
  //    all_on();
  //  }
  if (flag) {
    lcd.setCursor(9, 3);
    lcd.print("NRM /");
  }
  else {
    lcd.setCursor(9, 3);
    lcd.print("TST /");
  }


  /*--------------------------------------
    BT BT BT BT BT BT BT BT BT BT BT BT BT
    --------------------------------------*/
  if (Serial.available())
  {
    char data = Serial.read();
    if (data == '1')
    {
      void sens_u();
    }
    else if (data == '2')
    {
      void sens_d();
    }
    else if (data == '3')
    {
      lcd.setCursor(9, 3);
      lcd.print("TST /");
      all_on();
      flag = false;
    }

    else if (data == '4')
    {
      all_off();
      flag = true;
      lcd.setCursor(9, 3);
      lcd.print("NRM /");
    }
  }
}

/********************************************************************
  VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs
********************************************************************/
void writereg() {
  digitalWrite(latchPin, LOW);
  for (int i = 0; i < 65; i++) {
    digitalWrite(clockPin, LOW);
    digitalWrite(dataPin, registers[i]);
    digitalWrite(clockPin, HIGH);
  }

  digitalWrite(latchPin, HIGH);
}

void all_on() {
  for (int i = 64; i > 0; i--) {
    registers[i] = LOW;
    delay(50);
    writereg();
  }
}

void all_off() {
  for (int i = 64; i > 0; i--) {
    registers[i] = HIGH;
    delay(50);
    writereg();
  }
}

void sens_u() {
  for (int i = 0; i < 65; i++) {
    registers[i] = LOW;
    delay(st_);
    writereg();
  }
  delay(wt_);
  for (int i = 0; i < 65; i++) {
    registers[i] = HIGH;
    delay(st_);
    writereg();
  }
}

void sens_d() {
  for (int i = 64; i > 0; i--) {
    registers[i] = LOW;
    delay(st_);
    writereg();
  }
  delay(wt_);
  for (int i = 64; i > 0; i--) {
    registers[i] = HIGH;
    delay(st_);
    writereg();
  }
}
//___Dr.TRonik YouTube...
