//******************************************************//
//Klasa VTItime - informacja o aktualnej dacie i godzinie
//oraz milisekundy.
//******************************************************//
class VTItime
{
  private:
    volatile time_t        vtiDateTime;
    volatile unsigned long vtiMS;

  public:
    // Default constructor - creates a VTItime object
    // and initializes the member variables and state
    VTItime()
    {
      vtiDateTime = 0;
      vtiMS       = 0;
    }


    // Constructor - creates a VTItime object
    // and initializes the member variables and state
    VTItime(time_t DT, unsigned long ms)
    {
      vtiDateTime = DT;
      vtiMS       = ms;
    }

    // Set date, time and millis
    void setDateTime(time_t DT, unsigned long ms)
    {
      vtiDateTime = DT;
      vtiMS       = ms;
    }


    // Update every 1 ms
    void Update()
    {
      switch (vtiMS)
      {
        case 999:
          vtiMS        = 0;
          vtiDateTime += 1;
         break;

        default:
          vtiMS += 1;
         break;
      }
    }


    // get millis
    unsigned long getMillis()
    {
      return vtiMS;
    }


    // get DateTime
    time_t getDateTime()
    {
      return vtiDateTime;
    }


    // Zaokrąglij do pełnej sekundy
    void roundDateTime()
    {
      if (vtiMS >= 500) vtiDateTime += 1;
      vtiMS = 0;
    }

};
