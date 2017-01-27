#include <Time.h>


#ifndef _VTI_H
  #define _VTI_H


//******************************************************//
//Struktura przechowująca informacje o dacie, godzinie
// oraz milisekundach.
//******************************************************//
struct DateTimeMS
{
  volatile time_t        DT;
  volatile unsigned long MS;
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


    // Zwróć strukture vtiDateTimeMS - w przypadku gdy rejestr TCNT1
    // jest większy niż połowa OCR1A (powyżej 0,5ms) zaokrąglij
    // w górę do pełnej ms. 
    DateTimeMS  getDateTimeMS()
    {
      DateTimeMS tmpDateTimeMS;
      
      if( (TCNT1 << 1) > OCR1A )
      {                                     //Ułamek milisekund > 0,5ms - trzeba zaokrąglać w górę
          switch (vtiDateTimeMS.MS)
          {
            case 999:
              tmpDateTimeMS.MS = 0;
              tmpDateTimeMS.DT = vtiDateTimeMS.DT + 1;
            break;

            default:
              tmpDateTimeMS.MS = vtiDateTimeMS.MS + 1;
              tmpDateTimeMS.DT = vtiDateTimeMS.DT;
            break; 
          }
          return tmpDateTimeMS;
        } 
       else                                 //Ułamek milisekund <= 0,5ms - nie trzeba zaokrąglać
       {
        return vtiDateTimeMS;
       }
    }

};

#endif
