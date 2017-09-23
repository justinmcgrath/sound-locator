#include <Stepper.h>
#define SPEED_OF_SOUND 343 // meters/second

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


double sgn (double x) {
  return x < 1e-5 ? -1 : x > 1e-5;
}

class mover {
    int steps_;
    int pin1_;
    int pin2_;
    int pin3_;
    int pin4_;
    double max_block_time_; // milliseconds
    int max_steps_per_move;
    double degrees_per_step_; // minimum turn distance
    double rpms_;
  public:
    Stepper stepper_;
    double move_toward(double); // takes angle in degrees relative to current facing, returns degrees moved
    double get_min_turn_angle();
    int set_rpms();
    mover(int steps, int pin1, int pin2, int pin3, int pin4, int rpms, double max_block_time);
};

mover::mover(int steps, int pin1, int pin2, int pin3, int pin4, int rpms, double max_block_time) : steps_(steps), pin1_(pin1), pin2_(pin2), pin3_(pin3), pin4_(pin4), rpms_(rpms), max_block_time_(max_block_time), stepper_(Stepper(steps_, pin1_, pin2_, pin3_, pin4_)) {
  stepper_.setSpeed(rpms_);
  degrees_per_step_ = 360.0 / steps_;
  max_steps_per_move = (int)floor(max_block_time * rpms_ * steps_ / 60000); // steps_/rotation * rotations/min * 1 min / 60 s * 1 s / 1000 ms = steps / ms
  //  Serial.println("RPMS" + (String)rpms_);
}

double mover::get_min_turn_angle() {
  return degrees_per_step_;
}

double mover::move_toward(double angle) {
  //  Serial.println("angle" + (String)angle);
  //  Serial.println("degrees_per_step_" + (String)degrees_per_step_);
  int steps_to_move = (int)floor(angle / degrees_per_step_);
  //  Serial.println("Before" + (String)steps_to_move);
  double time_to_move = steps_to_move * 1 / (rpms_ * steps_ / .06); // 1/(steps_/rotation * rotations/min * 1min/60s * 1s/1000ms) = ms/step
  steps_to_move = (int)(sgn(steps_to_move) * min(abs(steps_to_move), abs(max_steps_per_move))); // account for negative angles
  //  Serial.println("After" + (String)steps_to_move);
  //  Serial.println("max_steps_per_move" + (String)max_steps_per_move);
  stepper_.step(steps_to_move);
  return steps_to_move * degrees_per_step_;
}

class fifo {
    int* array_;
    int size_;
    int zero_index_ = -1;
    int present_index_ = -1;
    int length_ = 0;
  public:
    void append(int x);
    int length() const;
    int operator[](int i) const;
    fifo(int n);
    ~fifo() {
      delete [] array_;
    }
};

fifo::fifo(int n) : size_(n)
{
  array_ = new int[size_];
}

int fifo::length() const
{
  return length_;
}

void fifo::append(int x)
{
  present_index_ = present_index_ < (size_ - 1) ? ++present_index_ : 0;
  array_[present_index_] = x;
  length_ = min(length_ + 1, size_);
  zero_index_ = present_index_ + 1 < length_ ? present_index_ + 1 : 0;
}

int fifo::operator[](int i) const
{
  int index = ((zero_index_ + i) % size_);
  return array_[index];
}

double get_audio_delay(const fifo& x, const fifo& y, int n, int max_delay);
double get_audio_delay(const fifo& x, const fifo& y, int n, int max_delay) {
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

stereo_locator::stereo_locator(int mic1_pin, int mic2_pin, unsigned long sample_freq, int sample_size, double mic_distance) : mic1_pin_(mic1_pin), mic2_pin_(mic2_pin), sample_freq_(sample_freq), sample_size_(sample_size), mic_distance_(mic_distance), mic1_sample_(fifo(sample_size)), mic2_sample_(fifo(sample_size)) {
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

void setup() {
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

void loop() {
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

