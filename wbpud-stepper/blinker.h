

class Blinker {

private:
  unsigned short blinks;
  unsigned long onDuration; // in msec
  unsigned long offDuration; // in msec
  void (*onFlashStart)() ;
  void (*onFlashStop)() ;
  void (*onBlinkEnd)() ;
  bool running;
  unsigned long blinkStartStamp;
  bool lit;
  unsigned short blinkNo;

public:
  Blinker(unsigned short blinksIn, unsigned long onDurationIn, unsigned long offDurationIn, void (*onFlashStartIn)() , void (*onFlashStopIn)() , void (*onBlinkEndIn)() ) {
    blinks = blinksIn;
    onDuration = onDurationIn;
    offDuration = offDurationIn;
    onFlashStart = onFlashStartIn;
    onFlashStop = onFlashStopIn;
    onBlinkEnd = onBlinkEndIn;
    running = false;
    lit = false;
  }

  void run() {
    if (!running) {
      blinkStartStamp = millis();
      blinkNo = 1;
      running = true;
      onFlashStart();
      lit = true;
    }

    if (millis() - blinkStartStamp > blinkNo * (onDuration + offDuration) && !lit) {
      blinkNo++;
      onFlashStart();
      lit = true;
    } else if (millis() - blinkStartStamp > (blinkNo - 1) * (onDuration + offDuration) + onDuration && lit) {
      onFlashStop();
      lit = false;
      if (blinkNo == blinks) {
        onBlinkEnd();
        running = false;
      }
    }
  }

  bool isRunning() {return running;}
};