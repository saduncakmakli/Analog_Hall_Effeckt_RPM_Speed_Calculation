#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

/*
Kilometer Per Hour(km/hr) = Wheel Diameter(cm) × Revolution Per Minute(RPM) × 0.001885
Revolution Per Hour = Revolution Per Minute(RPM)*60
Perimeter length(km) = Perimeter length(cm)/(100*1000)
Kilometer Per Hour(km/hr) = Revolution Per Minute(RPM)*60*Perimeter length(cm)/(100*1000)
Perimeter length = 2*PI*Wheel Diameter(cm)
*/
const double FORMUL_0 (60.0/100000.0); //Kilometer Per Hour(km/hr) = RPM*Perimeter length(cm)*FORMUL_0

void CanBroadcastSingleByteData(byte data);
void PrintSerial();
void PrintSerialRPM_Number_of_passes_based();
void PrintSerialMagnetPassCount();
void AddSetRPM(unsigned short RPM);
void PrintSerialSpeed_TimeBased();
void PrintSerialRPM_TimeBased();
void CalculateRPM_TimeBased();
void CalculateSpeed_TimeBased();
void DiziSifirAta(int dizi[], int dizi_buyukluk);
void DiziSifirAta(unsigned long dizi[], int dizi_buyukluk);
unsigned long DiziOrtalamaHesapla(unsigned long dizi[], int dizi_buyukluk);
unsigned long DiziOrtalamaHesapla(int dizi[], int dizi_buyukluk);
void HallEffectDetection();
void MagnetDetected();
void Calculate_rpm_number_of_passes_based();
void setup();
void loop();

//HALL EFFECT A0 PİNİ
const int HALL_ANALOG = A0;

//CANBUS MCP2515 SPI CS PIN
const int spiCSPin = 10;

//CAN INIT
MCP_CAN CAN(spiCSPin);

//SETTINGS-HIZ-CANBUS
unsigned long eskizaman_HIZ = 0;
unsigned long TIMERLOOP_HIZ_milisn = 87;
const int DATA_BYTE_HIZ = 1;
const byte CAN_ID_HIZ = 0x14; //int 20

//SETTINGS-Sabitler
const short const_hall_precision = 20; //Ne kadar büyükse o kadar hassas ANCAK !!! Bu değer sensörün yerleştirildiği sistemin mıknatıslara uzaklığına ve kusursuzluğuna göre ayarlanmalıdır! Aksi taktirde değeri büyütmek hatalı sonuç üretecektir. Mıknatıslar sensöre ne kadar yakın ve kusursuz dizildi ise hall_precision değeri o kadar büyütülebilir. Değer minimum 10 olmalıdır! Maksimum sınır 128dir.
const double wheel_diameter = 5.5; //cm centimeter //test düzeneği için 5.5
const double perimeter_length = 2*PI*wheel_diameter; //cm centimeter
const unsigned short miknatis_sayisi = 8;

//SETTINGS-Gecen Sure Bazlı Hesap Yapan Sabitler
const unsigned short size_of_average_time_based = 8; //En son kaç tane geçişin ortalamasının alınacağı
const unsigned short max_ms_control = 300; //İki mıknatıs arasında geçebilecek max süre
const unsigned short vehicle_can_broadcast_milisn = 50; // Araç dururken kaç msde bir hızı 0 olarak göndereceği
const int stop_detection_count = 10; //Kaç vehicle_can_broadcast_milisn süresinde mıknatıs algılanmazsa hızın sıfırlanacağı

//SETTINGS-Gecis Sayısı Bazlı Hesap Yapan Sabitler
const unsigned short calculate_milisn_number_of_passes_based = 1000;

//VARIABLES-Degiskenler
short hall_precision = const_hall_precision;
short control_magnet_pass_count;

//VARIABLES-Gecen Sure Bazlı Hesap Yapan Degiskenler
double speed_time_based = 0;
double rpm_time_based = 0;

unsigned long RpmDizisi[size_of_average_time_based];
unsigned short RpmDizisiKacinciEleman = 0;

bool hall_konum = false; //True Büyük, False Küçük
unsigned long hall_low_ms; //Hall efect sensor low okuduğu andaki ms
unsigned long hall_high_ms; //Hall efect sensor high okuduğu andaki ms
unsigned long hall_tek_gecis_ms;

bool stop_detection_debug_handler = false;
int stop_detection_counter = 0;
unsigned long stop_detection_eskizaman = 0;
unsigned short stop_detection_duration = 0;
byte stop_detection[stop_detection_count];

//Gecis Sayısı Bazlı Hesap Yapan Degiskenler
float rpm_number_of_passes_based;
unsigned long old_time__milisn_calculate_number_of_passes_based = 0;
unsigned short duration_number_of_passes_based = 0;


void setup()
{
  Serial.begin(9600);

  DiziSifirAta(RpmDizisi, size_of_average_time_based);

  //DEĞİŞTİRME!!
  if(hall_precision<10) hall_precision=10;
  else if(hall_precision>128) hall_precision=128;
  //DEĞİŞTİRME!!

  //CAN SETUP
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS init Failed");
    delay(100);
  }
  Serial.println("CAN BUS Shield Init OK!");
}

void loop()
{
  unsigned long simdiki_zaman = millis();
  if (simdiki_zaman - old_time__milisn_calculate_number_of_passes_based >= calculate_milisn_number_of_passes_based)
  {
    Calculate_rpm_number_of_passes_based();

    duration_number_of_passes_based = 0;
    old_time__milisn_calculate_number_of_passes_based = millis();
  }

  unsigned long stop_detection_simdiki_zaman = millis();
  if (stop_detection_simdiki_zaman - stop_detection_eskizaman >= vehicle_can_broadcast_milisn)
  {
    if (stop_detection_duration == 0)
    {
      if (stop_detection_counter < stop_detection_count)
      {
        stop_detection_counter++;
      }
      else if (stop_detection_counter == stop_detection_count)
      {
        stop_detection_debug_handler = true;
        if (stop_detection_debug_handler == true)
        {
          AddSetRPM(0);
          CalculateRPM_TimeBased();
          CalculateSpeed_TimeBased();
        }
      }
    }
    else
    {
      stop_detection_counter = 0;
    }
    stop_detection_duration = 0;
    stop_detection_eskizaman = millis();
    PrintSerial();
    //CAN BROADCAST
    CanBroadcastSingleByteData(speed_time_based);
  }
  HallEffectDetection();
}

void PrintSerial()
{
  //PrintSerialRPM_Number_of_passes_based();
  PrintSerialRPM_TimeBased();
  //PrintSerialSpeed_TimeBased();
  //PrintSerialMagnetPassCount();
}

void PrintSerialMagnetPassCount()
{
  Serial.print("PassCount: ");
  Serial.println(control_magnet_pass_count);
}

void PrintSerialRPM_TimeBased()
{
  Serial.print("RPM: ");
  Serial.println(rpm_time_based);
}

void PrintSerialRPM_Number_of_passes_based()
{
  Serial.print("-RPM:");
  Serial.println(rpm_number_of_passes_based);
}

void PrintSerialSpeed_TimeBased()
{
  Serial.print("SPEED: ");
  Serial.println(speed_time_based);
}

void CalculateRPM_TimeBased()
{
  rpm_time_based = (DiziOrtalamaHesapla(RpmDizisi,size_of_average_time_based));
}

void CalculateSpeed_TimeBased()
{
  speed_time_based = rpm_time_based*perimeter_length*FORMUL_0;
}

void Calculate_rpm_number_of_passes_based()
{
  rpm_number_of_passes_based = (duration_number_of_passes_based/8)*(1000/calculate_milisn_number_of_passes_based)*60;
}

void DiziSifirAta(int dizi[], int dizi_buyukluk)
{
  for (int sayac=0; sayac<dizi_buyukluk; sayac++)
  {
    dizi[sayac] = 0;
  }
}

void DiziSifirAta(unsigned long dizi[], int dizi_buyukluk)
{
  for (int sayac=0; sayac<dizi_buyukluk; sayac++)
  {
    dizi[sayac] = 0;
  }
}

unsigned long DiziOrtalamaHesapla(int dizi[], int dizi_buyukluk)
{
  long toplam = 0;
  for (int sayac = 0; sayac < dizi_buyukluk; sayac++)
  {
    toplam += dizi[sayac];
  }
  return (toplam/dizi_buyukluk);
}

unsigned long DiziOrtalamaHesapla(unsigned long dizi[], int dizi_buyukluk)
{
  double toplam = 0;
  for (int sayac = 0; sayac < dizi_buyukluk; sayac++)
  {
    toplam += dizi[sayac];
  }
  return (toplam/dizi_buyukluk);
}

void AddSetRPM(unsigned short RPMValue)
{
  RpmDizisi[RpmDizisiKacinciEleman] = RPMValue;

  if (RpmDizisiKacinciEleman != size_of_average_time_based-1)
  {
    RpmDizisiKacinciEleman++;
  }
  else
  {
    RpmDizisiKacinciEleman=0;
  }
}

void MagnetDetected()
{
  unsigned short hall_tek_gecis_rpm;
  duration_number_of_passes_based++; //Normal Hesap İçin Gerekli
  stop_detection_duration++; //Arac Durduğunda Hızı 0lamak için gerekli
  control_magnet_pass_count++; //Toplam kaç kere mıknatıs algıladığını tutar.
  stop_detection_debug_handler = false;
  hall_tek_gecis_rpm = (60000/(hall_tek_gecis_ms*miknatis_sayisi));
  if (hall_tek_gecis_ms > max_ms_control) hall_tek_gecis_rpm = 0;
  AddSetRPM(hall_tek_gecis_rpm);
  CalculateRPM_TimeBased();
  CalculateSpeed_TimeBased();
}

void HallEffectDetection()
{
  if (hall_konum)
  {
    if (analogRead(HALL_ANALOG)<512-hall_precision)
    {
      hall_konum = false;
      hall_low_ms = millis();
      hall_tek_gecis_ms = hall_low_ms - hall_high_ms;
      MagnetDetected();
    }
  }
  else
  {
    if (analogRead(HALL_ANALOG)>512+hall_precision)
    {
      hall_konum = true;
      hall_high_ms = millis();
      hall_tek_gecis_ms = hall_high_ms - hall_low_ms;
      MagnetDetected();
    }
  }
}

void CanBroadcastSingleByteData(byte data)
{
  byte stmp[DATA_BYTE_HIZ] = {data};
  CAN.sendMsgBuf(CAN_ID_HIZ, 0, DATA_BYTE_HIZ, stmp);
}