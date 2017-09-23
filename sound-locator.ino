#include "mover.h"
#include "fifo.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

double sgn (double x)
{
  return x < 1e-5 ? -1 : x > 1e-5;
}

double get_audio_delay(const fifo& x, const fifo& y, int n, int max_delay)
{
  /* Calculate the mean of the two series x[], y[] */
  double mx = 0;
  double my = 0;
  for (int i = 0; i < n; i++) {
    mx += x[i];
    my += y[i];
  }
  mx /= n;
  my /= n;

  /* Calculate the denominator */
  double sx = 0;
  double sy = 0;
  for (int i = 0; i < n; ++i) {
    double dx = x[i] - mx;
    double dy = y[i] - my;
    sx += dx * dx;
    sy += dy * dy;
  }
  double denom = sqrt(sx * sy);

  double max_correlation = 0;
  int shift_at_max_correlation = -max_delay;

  /* Calculate the correlation series */
  for (int shift = -max_delay; shift <= max_delay; ++shift) { // JAM "<" or "<=" ?
    double sxy = 0;
    for (int i = 0; i < n; ++i) {
      int j = i + shift;
      if (j < 0 || j >= n)
        continue;
      else
        sxy += (x[i] - mx) * (y[j] - my);
      /* Or should it be (?)
         if (j < 0 || j >= n)
         sxy += (x[i] - mx) * (-my);
         else
         sxy += (x[i] - mx) * (y[j] - my);
         */
    }
    double r = sxy / denom;
    //Serial.println("r  " + (String)r + " max" + (String)max_correlation + " max shift " + (String)shift_at_max_correlation);

    /* r is the correlation coefficient at "shift" */
    if (abs(r) > abs(max_correlation)) {
      max_correlation = r;
      shift_at_max_correlation = shift;
    }
  }
  return shift_at_max_correlation;
}

class stereo_locator {
    fifo mic1_sample_;
    fifo mic2_sample_;
    int mic1_pin_;
    int mic2_pin_;
    unsigned long sample_freq_;
    unsigned long last_sample_time_;
    unsigned long last_print_ = 0;
    int sample_size_;
    double mic_distance_;
  public:
    double locate();
    stereo_locator(int mic1_pin, int mic2_pin, unsigned long sample_freq, int sample_size, double mic_distance);
};

stereo_locator::stereo_locator(int mic1_pin, int mic2_pin, unsigned long sample_freq, int sample_size, double mic_distance) : mic1_pin_(mic1_pin), mic2_pin_(mic2_pin), sample_freq_(sample_freq), sample_size_(sample_size), mic_distance_(mic_distance), mic1_sample_(fifo(sample_size)), mic2_sample_(fifo(sample_size))
{
  last_sample_time_ = millis();
}

int loops = 0;
double stereo_locator::locate() {
  mic1_sample_.append(analogRead(mic1_pin_));
  mic2_sample_.append(analogRead(mic2_pin_));
  if (millis() - last_print_ > 5000) {
    for (int q = 0 ; q < mic1_sample_.length(); ++q) {
      Serial.print((String)mic1_sample_[q] + ", ");
    }
    Serial.print('\n');
    for (int q = 0 ; q < mic2_sample_.length(); ++q) {
      Serial.print((String)mic2_sample_[q] + ", ");
    }
    Serial.print('\n');
    Serial.println("mic length  " + String(mic1_sample_.length()));
    last_print_ = millis();
  }
  unsigned long current_time = millis();
  double sample_delay = -1; // Returns a negative value when there is no new delay to report.
  //Serial.println("current_time " + (String)current_time);
  //Serial.println("last time " + (String)last_sample_time_);
  if (current_time - last_sample_time_ >= sample_freq_) {
    sample_delay = get_audio_delay(mic1_sample_, mic2_sample_, mic1_sample_.length(), min(mic1_sample_.length() - 1, 39));
    //     Serial.println("time after get delay " + (String)millis());
    last_sample_time_ = current_time;
    //Serial.println("sample delay  " + String(sample_delay));
    //Serial.println("mic length  " + String(mic1_sample_.length()));

    double distance_to_source = SPEED_OF_SOUND * sample_delay * sample_freq_;
    distance_to_source = sgn(distance_to_source) * min(abs(distance_to_source), mic_distance_);
    //Serial.println("distance  " + String(distance_to_source));

    return acos(distance_to_source / mic_distance_) - PI / 2;
  } else {
    return -1;
  }


}

stereo_locator mic_locator = stereo_locator(A0, A1, 500, 200, 0.15);
mover stepper_mover = mover(2048, 8, 10, 9, 11, 14, 100);

double target_location = 0;
fifo a(3);
int i = 0;

void setup()
{
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);
  Serial.println("Start\n\n\nStart");
}

//void loop() {
//  double new_target = mic_locator.locate();
//  Serial.println("target " + String(new_target));
////  a.append(i);
////  for (int j = 0; j < a.length(); ++j) {
////          Serial.println("a " + (String)j + " " + (String)a[j]);
////  }
////  i++;
//
//  delay(500);
//}

void loop()
{
  double new_target = mic_locator.locate();
  //Serial.println("target " + String(new_target));;
  if (new_target >= 0) {
    target_location = new_target;
  }
  //Serial.println(target_location);
  if (abs(target_location) > PI / 180) {
    double change = stepper_mover.move_toward(target_location * 180 / PI) * PI / 180;
    //    Serial.println("change " + (String)change);
    target_location = target_location - change;
  }

}

