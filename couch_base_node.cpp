#include "Couch.h"
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <termios.h>
#include <fcntl.h>

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

#define NUM_AXIS 6

using namespace std;

struct js_event {
    unsigned int time;  /* event timestamp in milliseconds */
    short value;   /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

struct joy_status {
    int button[11];
    int axis[NUM_AXIS];
};


Couch *steve;
double sx, sy, srot, minSpeed;

int read_joystick_event(int joystick_fd, struct js_event *jse)
{
    int bytes;
    bytes = read(joystick_fd, jse, sizeof(*jse)); 
    if (bytes == -1)
        return 0;

    if (bytes == sizeof(*jse))
        return 1;

    cout << "Unexpected bytes from joystick: " << bytes << endl;
    return -1;
}

int get_joystick_status(int joystick_fd, struct joy_status *jss)
{
    int rc;
    struct js_event jse;
    if (joystick_fd < 0)
        return -1;

    while ((rc = read_joystick_event(joystick_fd, &jse) == 1)) {
        jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
        if (jse.type == JS_EVENT_AXIS) {
            if (jse.number < NUM_AXIS) {
                jss->axis[jse.number] = jse.value;
            }
        } else if (jse.type == JS_EVENT_BUTTON) {
            if (jse.number < 10 && (jse.value == 0 || jse.value == 1)) {
                jss->button[jse.number] = jse.value;
            }
        }
    }
    return 0;
}


static void printStatus(struct status *s) {
    cout << "Battery voltage:" <<  s->battery_voltage << endl;
}

static void printFault(struct fault *f) {
    cout << "Fault code: " << (char)f->fault_code << " source " << 
        f->fault_source << " max " << f->max_fault_val << endl;
} 

int cap(int a) {
    a = a < -32767 ? -32767 : a;
    a = a > 32767 ? 32767 : a;
    return a;
} 

double magic (double a,double b){
    return abs(a) < abs(b) ? a:b;
}

void move_motors(struct joy_status *js)
{
    double x = (double)js->axis[0] / SHRT_MAX;
    double y = -(double)js->axis[1] / SHRT_MAX;
    double rot = (double)js->axis[3] / SHRT_MAX;

    static double lfs = 0;
    static double rfs = 0;
    static double lbs = 0;
    static double rbs = 0;

    cout << x << " " << y << " " << rot << endl;

    double xDir = x > 0 ? 1 : x < 0 ? -1 : 0;
    double x1 = (pow(x, 2) * (1.0 - minSpeed) * xDir ) * sx + minSpeed * xDir * 0.9;
    double yDir = y > 0 ? 1 : y < 0 ? -1 : 0;
    double y1 = (pow(y, 2) * (1.0 - minSpeed) * yDir) * sy  + minSpeed * yDir * 0.9;
    double zDir = rot > 0 ? 1 : rot < 0 ? -1 : 0;
    double rot1 = (pow(rot, 2) * (1.0 - minSpeed) * zDir) * srot + minSpeed * zDir;

    double scl = 0.99;

    lfs = magic(lfs*scl + (1-scl)*cap(x1 + y1 + rot1),(double)cap(x1 + y1 + rot1));
    rfs = magic(rfs*scl + (1-scl)*cap(x1 - y1 + rot1),(double)cap(x1 - y1 + rot1));
    lbs = magic(lbs*scl + (1-scl)*cap(x1 - y1 - rot1),(double)cap(x1 - y1 - rot1));
    rbs = magic(rbs*scl + (1-scl)*cap(x1 + y1 - rot1),(double)cap(x1 + y1 - rot1));


    int lf = (int)lfs;
    int rf = (int)rfs;
    int lb = (int)lbs;
    int rb = (int)rbs;

    cout << "Setting couch motors to: " << lf << " " << rf << " " << lb << 
        " " << rb << endl;
    steve->setMotors(lf, rf, lb, rb);
}

#define COUCH_PORT 1
#define JOY_PORT 2

int main(int argc, char **argv)
{
    steve = new Couch(argv[COUCH_PORT]);
    cout << "Opening joystick " << argv[JOY_PORT] << endl;
    int joystick_fd = open(argv[JOY_PORT], O_RDONLY | O_NONBLOCK);
    if (joystick_fd < 0) return -1;

    sx = 20000.0;
    sy = 20000.0;
    srot = 10000.0;
    minSpeed = 0.1;

    struct joy_status jss;
    memset(&jss, 0, sizeof(struct joy_status));

    cout << "Starting couch driving" << endl;
    while (true) { // probably should change this to exit on key press
        // Publish status message
        /*struct status *s = steve->getStatus();
          printStatus(s);
          struct fault *f = steve->getFault();
          printFault(f);
          delete s;
          delete f;*/
        // Read the joystick
        int rc = get_joystick_status(joystick_fd, &jss);
        if (rc < 0) {
            cout << "Error reading joystick" << endl;
            continue;
        }
	if (jss.button[0]) {
	    cout << "Resetting error state" << endl;
	    steve->resetFault();
	}	
        // Send commands to the couch
        move_motors(&jss); 
	usleep(10000);
    }

    close(joystick_fd);
    //delete steve;
    return 0;
}

