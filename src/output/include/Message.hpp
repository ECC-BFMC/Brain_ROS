
/**
 * message.hpp - Header file
 * message converter functions declarations
 */

#ifndef _MESSAGE_HPP_
#define _MESSAGE_HPP_

#include <string>
#include <sstream>
#include <complex>
#include <stdio.h>

namespace message{

    //enum for defining the actions that can be performed 
    //NON - non-valid 
    typedef enum {SPED, STER, BRAK, PIDA, ENPB, PIDS, NONV}Actions;
    //the strings associated to each action
    static std::string ActionStrings[] = { "SPED", "STER", "BRAK" , "PIDA" , "ENPB" , "PIDS", "NONV"};

    std::string getTextForKey(int);
    std::string speed(float);
    std::string steer(float);
    std::string brake(float);
    std::string pida(bool);
    std::string enpb(bool);
    std::string pids(float,float,float,float);
    
    

    Actions     text2Key(std::string);
};

#endif
