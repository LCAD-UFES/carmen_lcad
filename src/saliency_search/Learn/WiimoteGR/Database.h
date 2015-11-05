#ifndef DATABASE_H
#define DATABASE_H

#include <string>
#include <vector>
#include "Learn/WiimoteGR/Quantizer.h"
#include "Learn/WiimoteGR/TimeSlot.h"
#include "Learn/WiimoteGR/HMMLib.h"
#include <sqlite3.h>

using namespace std;

namespace WiimoteGR{
    
    struct HMM;
    class Quantizer;
    class DefaultQuantizer;

    class Database{
    public:
        //lazy initialization
        static Database& Open();
        static void Close();
        
        /* main functions */
        void SaveGesture(const Gesture& gesture);
        void SaveObservationSequence(const TimeSlot& seq);
        //Save or replace if its gestureName,quantizer,modelStyle and trained are same as one HMM in database.
        void SaveHMM(const HMM& hmm);
        void LoadGestures(const char* gestureName, vector<Gesture>& gestureVec);
        void LoadObservationSequences(const char* gestureName, const Quantizer& quantizer, vector<TimeSlot>& seqVec);
        void LoadHMM(HMM& hmm);
        void LoadHMMs(const Quantizer& quantizer, const char* modelStyle, bool trained, vector<HMM>& hmmVec);
        void DeleteGestures(const char* gestureName);
        void DeleteObservationSequences(const char* gestureName, const Quantizer& quantizer);

    protected:
        //for singleton pattern
        Database();
        ~Database();
        Database(const Database&);
        Database& operator= (const Database&);

    private:
        enum ReservedSpaceForLoad{
            gestureVecSpace = 20,
            seqVecSpace = 20,
            hmmVecSpace = 10
        };

        //singleton instance
        static Database* dbInstance;

        //sqlite3 database
        sqlite3 *db;

        //whether database is opened
        bool dbOpened;
        //result code of sqlite3 functions
        int rc;
        //error message of sqlite3 functions
        char *zErrMsg;

        //statements
        sqlite3_stmt* saveGestureStmt;
        sqlite3_stmt* saveObservationSequenceStmt;
        sqlite3_stmt* isExistHMMStmt;
        sqlite3_stmt* updateHMMStmt;
        sqlite3_stmt* insertHMMStmt;
        sqlite3_stmt* loadGesturesStmt;
        sqlite3_stmt* loadObservationSequencesStmt;
        sqlite3_stmt* loadHMMStmt;
        sqlite3_stmt* loadHMMsStmt;
    };
}

#endif
