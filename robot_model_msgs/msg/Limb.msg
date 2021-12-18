# modes
int16 LIMP=0
int16 HOLDING=1
int16 BALANCE_SUPPORT=2
int16 STEPPING=3
int16 SEEKING=4
int16 MANIPULATING=5

# type
int16 GENERIC=0
int16 LEG=1
int16 ARM=2

string name
int16 mode
int16 type
bool supportive
bool supporting

EffectorState effector

