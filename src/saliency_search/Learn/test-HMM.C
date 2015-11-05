#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include "Image/Point2D.H"
#include "Image/Dims.H"
#include "Image/DrawOps.H"
#include "Learn/HMM.H"
#include "GUI/DebugWin.H"

std::vector<Point2D<int> > square(Point2D<int> pos, Dims dim)
{
  std::vector<Point2D<int> > lines;
  lines.push_back(Point2D<int>(pos.i,pos.j));
  lines.push_back(Point2D<int>(pos.i+dim.w(),pos.j));
  lines.push_back(Point2D<int>(pos.i+dim.w(),pos.j+dim.h()));
  lines.push_back(Point2D<int>(pos.i,pos.j+dim.h()));
  lines.push_back(Point2D<int>(pos.i,pos.j));
  return lines;
}

std::vector<Point2D<int> > triangle(Point2D<int> pos)
{
  std::vector<Point2D<int> > lines;
  lines.push_back(Point2D<int>(pos.i,pos.j));
  lines.push_back(Point2D<int>(pos.i+100,pos.j));
  lines.push_back(Point2D<int>(pos.i+100-50,pos.j+70));
  lines.push_back(Point2D<int>(pos.i,pos.j));

  return lines;
}

std::vector<Point2D<int> > applelogo()
{
  std::vector<Point2D<int> > lines;
  lines.push_back(Point2D<int>(7, 71));
  lines.push_back(Point2D<int>(17, 88));
  lines.push_back(Point2D<int>(19, 90));
  lines.push_back(Point2D<int>(27, 95)); 
  lines.push_back(Point2D<int>(29, 95));
  lines.push_back(Point2D<int>(38, 91));
  lines.push_back(Point2D<int>(39, 91));
  lines.push_back(Point2D<int>(55, 95));
  lines.push_back(Point2D<int>(56, 95));
  lines.push_back(Point2D<int>(61, 93));
  lines.push_back(Point2D<int>(62, 93));
  lines.push_back(Point2D<int>(74, 73)); 
  lines.push_back(Point2D<int>(74, 72));
  lines.push_back(Point2D<int>(65, 62));
  lines.push_back(Point2D<int>(65, 61));
  lines.push_back(Point2D<int>(63, 57));
  lines.push_back(Point2D<int>(63, 56));
  lines.push_back(Point2D<int>(65, 42));
  lines.push_back(Point2D<int>(66, 42));
  lines.push_back(Point2D<int>(71, 36));
  lines.push_back(Point2D<int>(70, 35));
  lines.push_back(Point2D<int>(68, 33));
  lines.push_back(Point2D<int>(67, 32));
  lines.push_back(Point2D<int>(53, 29));
  lines.push_back(Point2D<int>(52, 30));
  lines.push_back(Point2D<int>(45, 32));
  lines.push_back(Point2D<int>(44, 27));
  lines.push_back(Point2D<int>(56, 15));
  lines.push_back(Point2D<int>(56, 14));
  lines.push_back(Point2D<int>(57, 7));
  lines.push_back(Point2D<int>(56, 6));
  lines.push_back(Point2D<int>(53, 7));
  lines.push_back(Point2D<int>(52, 7));
  lines.push_back(Point2D<int>(40, 19)); 
  lines.push_back(Point2D<int>(40, 20));
  lines.push_back(Point2D<int>(42, 26));
  lines.push_back(Point2D<int>(44, 33));
  lines.push_back(Point2D<int>(25, 29));
  lines.push_back(Point2D<int>(24, 29));
  lines.push_back(Point2D<int>(17, 31));
  lines.push_back(Point2D<int>(15, 32));
  lines.push_back(Point2D<int>(6, 43));
  lines.push_back(Point2D<int>(5, 45));
  lines.push_back(Point2D<int>(5, 64));
  lines.push_back(Point2D<int>(6, 65));
  lines.push_back(Point2D<int>(7, 70));
  
  return lines;
}


std::vector<Point2D<double> > getVel(const std::vector<Point2D<int> >& lines)
{
  std::vector<Point2D<double> > vel;

  for(uint i=0; i<lines.size()-1; i++)
  {
    Point2D<int> dPos = lines[i+1]-lines[i];
    double mag = sqrt((dPos.i*dPos.i) + (dPos.j*dPos.j))/4;
    for(int j=0; j<int(mag+0.5); j++)
      vel.push_back(Point2D<double>(dPos/mag));
  }

  return vel;

}

int quantize(float x, float y, float z)
{
    int val = 0;

    //Get the magnitude 
    double rho = sqrt(x*x+y*y+z*z);

    if (rho>3.0)
      val = 3<<3;
    else if (rho > 2.0)
      val = 2<<3;
    else if (rho>1.0)
      val = 1<<3;
    else
      val = 0;

    if (x>y) val |= 1<<2; 
    if (y>z) val |= 1<<1; 
    if (z>x) val |= 1; 

    return val;
}

//Train an HMM with specific observations
HMM<uint> getHMM(const std::string& name, const std::vector<Point2D<int> >& lines)
{
  //Add an hmm with 5 states and 32 possible observations
  std::vector<uint> states; //5 States
  for(uint i=0; i<5; i++) 
    states.push_back(i);

  std::vector<uint> posibleObservations; //32
  for(uint i=0; i<32; i++) 
    posibleObservations.push_back(i);

  HMM<uint> hmm(states, posibleObservations, name);

  //Set the default transitions
  hmm.setStateTransition(0, 0, 0.5);
  hmm.setStateTransition(0, 1, 0.5);
  hmm.setStateTransition(1, 1, 0.5);
  hmm.setStateTransition(1, 2, 0.5);
  hmm.setStateTransition(2, 2, 0.5);
  hmm.setStateTransition(2, 3, 0.5);
  hmm.setStateTransition(3, 3, 0.5);
  hmm.setStateTransition(3, 4, 0.5);
  hmm.setStateTransition(4, 4, 1);

  //set the initial sstate
  hmm.setCurrentState(0, 1); //We start at the first state

  //Quantize the acc values into 32 numbers to represent the observations

  std::vector<Point2D<double> > vel = getVel(lines);

  std::vector< std::vector<uint> > observations;
  for(size_t j=0; j<vel.size(); j++)
  {
    std::vector<uint> observation;
    printf("InputValue ");
    for(size_t i=0; i<vel.size(); i++)
    {
      uint value = quantize(vel[(i+j)%vel.size()].i,
                          vel[(i+j)%vel.size()].j, 0);
      printf("%i ", value);
      observation.push_back(value);
    }
    printf("\n");
    observations.push_back(observation);
  }
  LINFO("Train");
  hmm.train(observations, 50);
  LINFO("Done");

  //hmm.show(); //Show the internal state of the HMM

  return hmm;
}

int main()
{

  //Testing the viterbi algorithm from wikipidia
  {
    //The posible states we can be in
    std::vector<std::string> states;
    states.push_back("Rainy");
    states.push_back("Sunny");

    //The posible observations, observed in each state
    std::vector<std::string> posibleObservations;
    posibleObservations.push_back("walk");
    posibleObservations.push_back("shop");
    posibleObservations.push_back("clean");

    //Initialize the hmm
    HMM<std::string> hmm(states, posibleObservations);

    //The Transition probability matrix;
    hmm.setStateTransition("Rainy", "Rainy", 0.7);
    hmm.setStateTransition("Rainy", "Sunny", 0.3);
    hmm.setStateTransition("Sunny", "Rainy", 0.4);
    hmm.setStateTransition("Sunny", "Sunny", 0.6);


    //////The state emission probability
    hmm.setStateEmission("Rainy", "walk", 0.1);
    hmm.setStateEmission("Rainy", "shop", 0.4);
    hmm.setStateEmission("Rainy", "clean", 0.5);
    hmm.setStateEmission("Sunny", "walk", 0.6);
    hmm.setStateEmission("Sunny", "shop", 0.3);
    hmm.setStateEmission("Sunny", "clean", 0.1);

    //Set our current state
    hmm.setCurrentState("Rainy", 0.6);
    hmm.setCurrentState("Sunny", 0.4);

    std::vector<std::string> observations;
    observations.push_back("walk");
    observations.push_back("shop");
    observations.push_back("clean");

    double prob;
    std::vector<std::string> path = hmm.getLikelyStates(observations, prob); 
    LINFO("FinalState prob %f", prob);
    printf("Path: ");
    for(uint i=0; i<path.size(); i++)
      printf("%s ", path[i].c_str());
    printf("\n");
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////
  {
    //std::vector<Point2D<int> > lines = square(Point2D<int>(50,50), Dims(150,50));
    //std::vector<Point2D<int> > lines = triangle(Point2D<int>(50,50));
    std::vector<Point2D<int> > lines = applelogo(); //triangle(Point2D<int>(50,50));

    Image<PixRGB<byte> > img(320,240,ZEROS);
    for(size_t i=0; i<lines.size()-1; i++)
      drawLine(img, lines[i], lines[i+1], PixRGB<byte>(255,0,0));

    std::vector<Point2D<double> > vel = getVel(lines);
    //Show the vel
    Point2D<double> pos(150,150);
    for(uint i=0; i<vel.size(); i++)
    {
      if (img.coordsOk(Point2D<int>(pos)))
          img.setVal(Point2D<int>(pos), PixRGB<byte>(0,255,0));
      //LINFO("V: %f %f P: %f %f", vel[i].i, vel[i].j, pos.i, pos.j);
      pos += vel[i];
    }

    SHOWIMG(img);


    //Test the HMM for sequence recognition 
    std::vector<HMM<uint> > hmms;

    hmms.push_back(getHMM("Square", square(Point2D<int>(50,50), Dims(50,50))));
    hmms.push_back(getHMM("Triangle", triangle(Point2D<int>(50,50))));

    LINFO("Test the hmm");
    lines = triangle(Point2D<int>(70,70));
    //lines = square(Point2D<int>(5,5), Dims(100,150));
    vel = getVel(lines);

    for(uint j=0; j<vel.size(); j++)
    {
      std::vector<uint> observations; 
      LINFO("Observations");
      for(size_t i=0; i<vel.size(); i++)
      {
        uint value = quantize(vel[(i+j)%vel.size()].i,
                            vel[(i+j)%vel.size()].j, 0);
        printf("%i ", value);
        observations.push_back(value);
      }
      printf("\n");

      //Check each HMM to see if it has the probability of being the sequence
      double maxProb = -1e100;
      std::string name = "";

      for(size_t i=0; i<hmms.size(); i++)
      {
        double prob = hmms[i].forward(observations);
        LINFO("HMM %s prob %e", hmms[i].getName().c_str(), exp(prob));
        if (prob > maxProb)
        {
          maxProb = prob;
          name = hmms[i].getName();
        }
      }

      LINFO("Max is %s", name.c_str());
    }
  }

}
