#include "Robots/SeaBeeIII/Localization.H"
#include "Transport/FrameInfo.H"
#include "Image/ColorOps.H"

// ######################################################################
/**
 *        \fun LocaLization Constructor function
 *        Initializes the Localization class
 *
 *
 *
 */
        Localization::Localization(OptionManager& mgr, const std::string& descrName, const std::string& tagName) :
          RobotBrainComponent(mgr, descrName, tagName),
          itsOfs(new OutputFrameSeries(mgr)),
          itsRunning(true),
          itsNumParticles(1000),
          itsKeepParticles(1000),
          generator(42u),
          generate_drift(generator, dist_type(M_DRIFT,SIGMA_DRIFT)),
          generate_trans(generator, dist_type(M_TRANS,SIGMA_TRANS))
        {
                //// The images used for plotting the particles
                iMap = Image<float > (2000,2000,ZEROS);
                originalMap = Image<float > (2000,2000,ZEROS);

                addSubComponent(itsOfs);
                initPoints(itsNumParticles);
        }
/**
 *        \fun registerTopics
 *        For registering the messages that we want to listen to and publish
 *
 *
 *
 */
void Localization::registerTopics()
{
  LINFO("Registering Localization Message");
  //this->registerPublisher("RetinaMessageTopic");
  this->registerSubscription("ObstacleMessageTopic");
  this->registerSubscription("OrientationMessageTopic");
}
/**
 *        \fun initPoints
 *        initialize the points in the image
 *
 *
 *
 */
void Localization::initPoints(const short int no = 1000){

                pList = vector<pParticle>();

                pParticle p;
                for (int i = 0 ; i < no ; i++){
                        p = pParticle(1000,0);
                        p.setProb(1.0);
                        pList.push_back(p);
                }
                itsNumParticles = no;
        }
/**
 *        \fun getNormalizer
 *        Returns the total probability of all the particles
 *
 *
 *
 */
        float Localization::getNormalizer(){

                float totalProb = 0.0;

                for (int i = 0 ; i < itsNumParticles ; i++){
                        totalProb += pList[i].getProb();
                        iMap.setVal( pList[i].p.j, pList[i].p.i, 255*pList[i].getProb() );
                }

                return totalProb;
        }
/**
 *        \fun evolve
 *
 */        void Localization::evolve()
        {
                return;
        }

// ######################################################################
/**
 *        \fun updateMessage
 *        Returns the total probability of all the particles
 *
 *
 *
 */
void Localization::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  if(eMsg->ice_isA("::RobotSimEvents::OrientationMessage"))
  {

          RobotSimEvents::OrientationMessagePtr oMsg = RobotSimEvents::OrientationMessagePtr::dynamicCast(eMsg);
        LINFO("Message is a camera config message: orientation of sub = %d, sub running status = %d", oMsg->orientation, oMsg->running);

        // First check if the population has depleted or not ... if it has then resample
        if(itsNumParticles < BETA * INIT_PARTICLES)
                resample(INIT_PARTICLES);

        predict(oMsg);

  }else if(eMsg->ice_isA("::RobotSimEvents::ObstacleMessage")) {

    RobotSimEvents::ObstacleMessagePtr oMsg = RobotSimEvents::ObstacleMessagePtr::dynamicCast(eMsg);
    LINFO("Message is a camera config message: camera1 distance = %d, orientation = %d", oMsg->distanceOfObstacle1, oMsg->orientation1);

    update(oMsg);

  }
}
/**
 *        \fun predict
 *        predict the new position based on the updates received from the compass
 *
 *
 *
 */

void Localization::predict(RobotSimEvents::OrientationMessagePtr oMsg){
// This function will contain the code for predicting the next position of points using gaussian distribution in
// for the error in their movement


        if(oMsg->running){
        // sub is moving forward ... let's predict the sub's position with the model that we have
                // so we will use orientation from the message and forward the points in that direction according to the time elapsed

                time_t newtime,elapsedTime;
                time(&newtime);
                elapsedTime = newtime - timestamp ;

                if( elapsedTime == 0 )
                        return;
                else {
                        //clear the image
                        iMap = Image<float > (2000,2000,ZEROS);

                        // First retrieve the distance travelled in current orientation
                        int distance = elapsedTime * BOT_SPEED;
                        int changeInOrientation = curOrientation - oMsg->orientation;

                        float Edrift=0, Etrans=0;
                        // Now let's run the particles through the prediction equations to see their new positions
                        // There are two sources of error for forward movement. One is distance and the other one is orientation
                        // We will have to model errors in both of them.
                        for (std::vector<class pParticle>::iterator iter = pList.begin(); iter != pList.end() ; iter++ )
                        {
                                ///  First erase the current position of the point
                                iMap.setVal((iter->p).j, (iter->p).i, 0.0 );
                                // Do the update on each particles using the equation
                                // SIGMA_TRANS = SIGMA_TRANS * sqrt(EXPERIMENT_DIST) // EXPERIMENT_DIST in centimeters preferable
                                // SIGMA_DRIFT = SIGMA_DRIFT * sqrt(EXPERIMENT_DIST/2.0)
                                // We will consider SIGMA_TRANS and SIGMA_DRIFT on a unit distance
                                // here rand_N is a random number generator, yet to be implemented
                                Etrans = generate_trans() * distance; //rand_N (M_TRANS * distance, SIGMA_TRANS * distance);

                                //This is for a completely different purpose. here we know that it we rotated certain degrees... but this thing just models any error
                                // in the measures taken by compass ... so set the distributions accordingly
                                Edrift = generate_drift() * changeInOrientation; //rand_N (M_DRIFT * changeInOrientation, SIGMA_DRIFT * changeInOrientation);

                                // First adjust for the drift
                                iter->orientation = iter->orientation + Edrift;
                                // Now adjust for the error in distance travelled
                                (iter->p).i = (iter->p).i + (distance+Etrans)*cos(iter->orientation);
                                (iter->p).j = (iter->p).j + (distance+Etrans)*sin(iter->orientation);
                                // One more iteration to calculate the drift in orientation

                                Edrift = generate_drift() * changeInOrientation; //rand_N (M_DRIFT * distance, SIGMA_TRANS * distance);

                                iter->orientation = iter->orientation + Edrift;
                                // set the point in the image
                                iMap.setVal( (iter->p).j, (iter->p).i, 255*iter->getProb() );
                        }
                }

        } else {
                // if it is possible to be not moving and still changing the orientation then do add an orientation change loop here.

                // if we are not moving, then just update the timestamp... it is just a message to say that we are not moving right now.
                        time(&timestamp);
        }

        //
        getCurrentEstimate();
        iMap.setVal( maxPoint.p.j, maxPoint.p.i, 255);

        // we have estimate in maxpoint ... now plot it on the map

        // write the image out to the map
        itsOfs->writeRGB(iMap, "iMap", FrameInfo("IMap", SRC_POS));
        itsOfs->writeRGB(luminance(iMap), "B/W_iMap", FrameInfo("B/W_IMap", SRC_POS));
        itsOfs->updateNext();
}

/**
 *        \fun update
 *        update the position stats according to the obstacle position
 *
 */
void Localization::update(RobotSimEvents::ObstacleMessagePtr oMsg ){
// change the ObstacleMessageTopic to LandmarkMessageTopic
// First we will obtain the point based on obstacle message
Point2D p = maxPoint;
p.j = p.j + sin(oMsg->orientation) * distanceOfObstacle;
p.i = p.i + cos(oMsg->orientation) * distanceOfObstacle;
//maxPoint.p.j maxPoint.p.i


}
/**
 *        \fn void Localization::resample(int no)
 *        resample the population and remove the unrelevant particles
 *
 */
void Localization::resample(int no){

// Calculate the mean of the population
        if(no != 0)
         itsKeepParticles = no;

        float totalProb=getNormalizer();
        float mean = totalProb/itsNumParticles;
        float stddev = 0.0 ;
        float minProb = 0.0 ;
        for (std::vector<class pParticle>::iterator iter = pList.begin(); iter != pList.end() ; iter++ )
        {
                stddev += ( iter->prob - mean ) * ( iter->prob - mean );
                if ( iter -> prob < minProb )
                        minProb = iter->prob;

        }
        stddev = stddev / itsNumParticles;
        stddev = sqrt ( stddev );

        // Now since we have standard deviation and mean at hand we can calculate what particles to say good bye to
        // if particles' probability is less than mean - ( (stddev + (mean - minProb - stddev)) / 2 ) and not greater than 0.01, then we will discard them

        for (std::vector<class pParticle>::iterator iter = pList.begin(); iter != pList.end() ; iter++ )
        {
                if( iter -> prob < 0.01 &&  ( iter -> prob < mean - (stddev + (mean - minProb - stddev ) )/2.0 ) )
                {
                        pList.erase(iter);
                        itsNumParticles --;
                }

        }

        std::sort(pList.begin(), pList.end(), greater<pParticle>() );

}
/**
 *        \fun getCurrentEstimate
 *        Return the current estimate
 *
 */
class pParticle& Localization::getCurrentEstimate(){
                // We can use several method for estimating position.
                // This implementation is using max
                        float maxProb = 0;
                        std::vector<class pParticle>::iterator iterEst;
                        for (std::vector<class pParticle>::iterator iter = pList.begin(); iter != pList.end() ; iter++ )
                        {
                                if(iter->prob > maxProb)
                                {
                                        maxProb = iter->prob;
                                        iterEst = iter;
                                }
                        }
                        return *iterEst;
                // Here I have used weighted mean
/*
                        float maxProb = 0;
                        float x=0,y=0;
                        for (std::vector<class pParticle>::iterator *iter = pList.begin(); iter != pList.end() ; iter++ )
                        {
                                x += (iter->p).x * iter->prob;
                                y += (iter->p).y * iter->prob;
                        }
                        return ( new pParticle(x,y) );
*/

}

/**
 *        \fun ~Localization()
 *        Destructor for the Localization class.
 *        Returns the total probability of all the particles
 *
 */
Localization::~Localization()
{
        // delete all the left out particles from memory
        pList.clear();
}



//Image<float>::iterator itr = a.beginw(), stop = a.endw();
//while (itr != stop)
  //*itr++ = 42;

/*
for(Image<float>::iterator itr = a.beginw(), stop = a.endw();
    itr != stop;
    ++itr)
  {
    *itr = 42;
  }

*/

/*                struct timeval newtime;
                long double elapsedTime;

                gettimeofday (&newtime,NULL);
                   elapsedTime = newtime.tv_usec - timestamp.tv_usec;
                elapsedTime = elapsedTime / 1000000.0; //convert into seconds

                printf("%LF - ",elapsedTime);

                if (elapsedTime * BOT_SPEED)*/

