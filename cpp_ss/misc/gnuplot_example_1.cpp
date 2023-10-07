
#include "gnuplot-iostream.h"
// #include "logfile.h"

#include <cstdlib>
#include <string>
#include <cmath>

// create gnuplot with channel identifiers strings
Gnuplot<std::string> gs;

// create gnuplot with channel identifiers int
Gnuplot<int> gi(100); // only 100 data points

// LogFile<std::string> logfile("example.log");

int main(){

    // send normal commands
    gs.command("set zeroaxis");

    // add channels
    gs.addChannel("random"); // title is intern created by { strstream s; s << "random"; } 
    gs.addChannel("sin","sinus","points");

    // logfile.addChannel("random");
    // logfile.addChannel("sin");
        
    // add six channels 
    for(int i=0;i<6;i++)
        gi.addChannel(i);
    
    double t=0;
    for(int i=0;i<500;i++){
    
        gs.putData("random",double(rand())/RAND_MAX);
        gs.putData("sin",sin(t));
        gs.plot();

        // logfile.putData("random",double(rand())/RAND_MAX);
        // logfile.putData("sin",sin(t));
        // logfile.print();
        
    
        for(int j=0;j<6;j++)
            gi.putData(j,sin(t+j/6.0*2*M_PI));

        gi.plot(); // plot all 2D
        //gi.plotXY(0,2); // plot 0 against 2 in XY-plot
        int x[]={0,3,5};
        int y[]={2,4,2};
        //gi.plotXY(x,y,3); // plot 0 against 2, 3 against 4, and 5 against 2 in XY-plot
        
        t+=0.1;
        usleep(int(0.02*1000000));
        
    }



    return 0;
}
