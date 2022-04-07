# adapted from Chimera SBS notations. Trying to be POSIX compliant
uint8   SPAWN   = 0
uint8   REINIT  = 2
uint8   KILL    = 9
uint8   WAKEUP  = 14
uint8   TERM    = 15
uint8   ON      = 18
uint8   CLEAR   = 19
uint8   OFF     = 20

time    stamp
string  module
uint8   request
