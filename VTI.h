#include <Time.h>


#ifndef _VTI_H
  #define _VTI_H


//******************************************************//
//Struktura przechowująca informacje o dacie, godzinie
// oraz milisekundach.
//******************************************************//
struct DateTimeMS
{
  time_t        DT;
  unsigned long MS;
};


//******************************************************//
//Klasa VTIclock - informacja o aktualnej dacie i godzinie
//oraz milisekundy.
//******************************************************//
class VTIclock
{
  private:
    DateTimeMS vtiDateTimeMS;

  public:
    // Default constructor - creates a VTIclock object
    // and initializes the member variables and state
    VTIclock()
    {
      vtiDateTimeMS.DT = 0;
      vtiDateTimeMS.MS = 0;
    }


    // Constructor - creates a VTIclock object
    // and initializes the member variables and state
    VTIclock(time_t DT, unsigned long ms)
    {
      vtiDateTimeMS.DT = DT;
      vtiDateTimeMS.MS = ms;
    }

    // Set date, time and millis
    void setDateTime(time_t DT, unsigned long ms)
    {
      vtiDateTimeMS.DT = DT;
      vtiDateTimeMS.MS = ms;
    }


    // Update every 1 ms
    void Update()
    {
      switch (vtiDateTimeMS.MS)
      {
        case 999:
          vtiDateTimeMS.MS  = 0;
          vtiDateTimeMS.DT += 1;
         break;

        default:
          vtiDateTimeMS.MS += 1;
         break;
      }
    }


    // get millis
    unsigned long getMillis()
    {
      return vtiDateTimeMS.MS;
    }


    // get DateTime
    time_t getDateTime()
    {
      return vtiDateTimeMS.DT;
    }


    // Zaokrąglij do pełnej sekundy
    void roundDateTime()
    {
      if (vtiDateTimeMS.MS >= 500) vtiDateTimeMS.DT += 1;
      vtiDateTimeMS.MS = 0;
    }


    // Zwróć strukture vtiDateTimeMS
    DateTimeMS  getDateTimeMS()
    {
      return vtiDateTimeMS;
    }

};

#endif
