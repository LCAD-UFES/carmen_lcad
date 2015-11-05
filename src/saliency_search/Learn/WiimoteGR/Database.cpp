#include <iostream>

#include "Learn/WiimoteGR/Database.h"
#include "Learn/WiimoteGR/TimeSlot.h"
#include "Learn/WiimoteGR/Quantizer.h"
#include "Learn/WiimoteGR/HMMLib.h"

namespace WiimoteGR{

    /* a trick to ensure Database will be closed and deleted on program termination */
    class DBCloser{
    public:
        ~DBCloser(){
            Database::Close();
        }
    } DBCloserInstance;

    Database* Database::dbInstance = 0;

    Database& Database::Open()
    {
        if(dbInstance == 0){
            dbInstance = new Database;
        }
        return *dbInstance;
    }

    void Database::Close()
    {
        if(dbInstance!=0){
            sqlite3_close(dbInstance->db);
            delete dbInstance;
            dbInstance = 0;
        }
    }

    /*
    main functions
    */
    void Database::SaveGesture(const Gesture& gesture)
    {
        //binding values
        rc = sqlite3_bind_text(saveGestureStmt,1, gesture.gestureName.c_str(), gesture.gestureName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in SaveGesture()" << std::endl << sqlite3_errmsg(db) << std::endl;
        rc = sqlite3_bind_int(saveGestureStmt,2, gesture.data.size());
        if(rc!=SQLITE_OK) std::cout << "Error when binding data in SaveGesture()" << std::endl << sqlite3_errmsg(db) << std::endl;
        rc = sqlite3_bind_blob(saveGestureStmt,3, &(gesture.data[0]), sizeof(Acceleration)*gesture.data.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding data in SaveGesture()" << std::endl << sqlite3_errmsg(db) << std::endl;

        //evaluating
        rc = sqlite3_step(saveGestureStmt);
        if(rc!=SQLITE_DONE) std::cout << "Error when evaluating in SaveGesture()" << std::endl << sqlite3_errmsg(db) << std::endl;

        //reset statement for next evaluation
        sqlite3_reset(saveGestureStmt);
        sqlite3_clear_bindings(saveGestureStmt);
    }

    void Database::SaveObservationSequence(const TimeSlot& seq)
    {
        //binding values
        rc = sqlite3_bind_text(saveObservationSequenceStmt, 1, seq.gestureName.c_str(), seq.gestureName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in SaveObservationSequence()" << std::endl;
        rc = sqlite3_bind_text(saveObservationSequenceStmt, 2, seq.quantizerName.c_str(), seq.quantizerName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding quantizerName in SaveObservationSequence()" << std::endl;
        rc = sqlite3_bind_int(saveObservationSequenceStmt, 3, seq.o.size());
        if(rc!=SQLITE_OK) std::cout << "Error when binding o in SaveObservationSequence()" << std::endl;
        rc = sqlite3_bind_blob(saveObservationSequenceStmt, 4, &(seq.o[0]), sizeof(size_t)*seq.o.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding o in SaveObservationSequence()" << std::endl;

        //evaluating
        rc = sqlite3_step(saveObservationSequenceStmt);
        if(rc!=SQLITE_DONE) std::cout << "Error when evaluating in SaveObservationSequence()" << std::endl;

        //reset statement for next evaluation
        sqlite3_reset(saveObservationSequenceStmt);
        sqlite3_clear_bindings(saveObservationSequenceStmt);
    }

    void Database::SaveHMM(const HMM& hmm)
    {
        /* testing existence*/
        //binding values
        rc = sqlite3_bind_text(isExistHMMStmt, 1, hmm.gestureName.c_str(), hmm.gestureName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in SaveHMM() for test existence" << std::endl;
        rc = sqlite3_bind_text(isExistHMMStmt, 2, hmm.quantizerName.c_str(), hmm.quantizerName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding quantizerName in SaveHMM() for test existence" << std::endl;
        rc = sqlite3_bind_text(isExistHMMStmt, 3, hmm.modelStyle.c_str(), hmm.modelStyle.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding modelStyle in SaveHMM() for test existence" << std::endl;
        rc = sqlite3_bind_int(isExistHMMStmt, 4, hmm.trained);
        if(rc!=SQLITE_OK) std::cout << "Error when binding trained in SaveHMM() for test existence" << std::endl;

        sqlite3_stmt* chosenStmt = 0;

        //evaluating
        rc = sqlite3_step(isExistHMMStmt);
        if(rc==SQLITE_DONE){
            //not exist, insert
            chosenStmt = insertHMMStmt;
        }else if(rc==SQLITE_ROW){
            //exist, update
            chosenStmt = updateHMMStmt;
        }else{
            std::cout << "Error when evaluating in SaveHMM() for test existence" << std::endl;
        }
        //reset
        sqlite3_reset(isExistHMMStmt);
        sqlite3_clear_bindings(isExistHMMStmt);


        /* INSERT or UPDATE */
        if(chosenStmt != 0){
            //binding values
            rc = sqlite3_bind_text(chosenStmt, 1, hmm.gestureName.c_str(), hmm.gestureName.size(), SQLITE_TRANSIENT);
            if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in SaveHMM()" << std::endl;
            rc = sqlite3_bind_text(chosenStmt, 2, hmm.quantizerName.c_str(), hmm.quantizerName.size(), SQLITE_TRANSIENT);
            if(rc!=SQLITE_OK) std::cout << "Error when binding quantizerName in SaveHMM()" << std::endl;
            rc = sqlite3_bind_text(chosenStmt, 3, hmm.modelStyle.c_str(), hmm.modelStyle.size(), SQLITE_TRANSIENT);
            if(rc!=SQLITE_OK) std::cout << "Error when binding modelStyle in SaveHMM()" << std::endl;
            rc = sqlite3_bind_int(chosenStmt, 4, hmm.trained);
            if(rc!=SQLITE_OK) std::cout << "Error when binding trained in SaveHMM()" << std::endl;
            rc = sqlite3_bind_int(chosenStmt, 5, hmm.N);
            if(rc!=SQLITE_OK) std::cout << "Error when binding N in SaveHMM()" << std::endl;
            rc = sqlite3_bind_int(chosenStmt, 6, hmm.M);
            if(rc!=SQLITE_OK) std::cout << "Error when binding M in SaveHMM()" << std::endl;
            rc = sqlite3_bind_blob(chosenStmt, 7, &(hmm.pi[0]), sizeof(double)*hmm.pi.size(), SQLITE_TRANSIENT);
            if(rc!=SQLITE_OK) std::cout << "Error when binding pi in SaveHMM()" << std::endl;
            rc = sqlite3_bind_blob(chosenStmt, 8, &(hmm.A[0]), sizeof(double)*hmm.A.size(), SQLITE_TRANSIENT);
            if(rc!=SQLITE_OK) std::cout << "Error when binding A in SaveHMM()" << std::endl;
            rc = sqlite3_bind_blob(chosenStmt, 9, &(hmm.B[0]), sizeof(double)*hmm.B.size(), SQLITE_TRANSIENT);
            if(rc!=SQLITE_OK) std::cout << "Error when binding B in SaveHMM()" << std::endl;

            //evaluating
            rc = sqlite3_step(chosenStmt);
            if(rc!=SQLITE_DONE) std::cout << "Error when evaluating in SaveHMM()" << std::endl;

            //reset statement for next evaluation
            sqlite3_reset(chosenStmt);
            sqlite3_clear_bindings(chosenStmt);
        }
    }

    void Database::LoadGestures(const char* gestureName, vector<Gesture>& gestureVec)
    {
        gestureVec.reserve(gestureVecSpace);
        //SQL: "SELECT dataLength data FROM GestureTable WHERE gestureName==?1;"
        rc = sqlite3_bind_text(loadGesturesStmt, 1, gestureName, -1, SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in LoadGestures()" << std::endl;

        Gesture tempGesture(gestureName);
        size_t dataLength;
        const Acceleration* dataPtr = 0;

        //evaluation
        rc = sqlite3_step(loadGesturesStmt);
        while(rc==SQLITE_ROW){
            gestureVec.push_back(tempGesture);
            dataLength = sqlite3_column_int(loadGesturesStmt, 0);
            dataPtr = (const Acceleration*)sqlite3_column_blob(loadGesturesStmt, 1);
            gestureVec.back().data.assign(dataPtr,dataPtr+dataLength);
            rc = sqlite3_step(loadGesturesStmt);
        }
        if(rc!=SQLITE_DONE) std::cout << "Error when evaluation in LoadGestures()" << std::endl;
        //reset
        sqlite3_reset(loadGesturesStmt);
        sqlite3_clear_bindings(loadGesturesStmt);
    }

    void Database::LoadObservationSequences(const char* gestureName, const Quantizer& quantizer, vector<TimeSlot>& seqVec)
    {
        seqVec.reserve(seqVecSpace);
        //SQL: "SELECT oLength o FROM ObservationSequenceTable WHERE gestureName==?1 AND quantizerName==?2;";
        rc = sqlite3_bind_text(loadObservationSequencesStmt, 1, gestureName, -1, SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in LoadObservationSequences()" << std::endl;
        rc = sqlite3_bind_text(loadObservationSequencesStmt, 2, quantizer.name.c_str(), quantizer.name.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding quantizerName in LoadObservationSequences()" << std::endl;

        TimeSlot tempSeq(gestureName,quantizer);
        size_t oLength;
        const size_t* oPtr = 0;

        //evaluation
        rc = sqlite3_step(loadObservationSequencesStmt);
        while(rc==SQLITE_ROW){
            oLength= sqlite3_column_int(loadObservationSequencesStmt, 0);
            oPtr = (const size_t*)sqlite3_column_blob(loadObservationSequencesStmt, 1);
            seqVec.push_back(tempSeq);
            seqVec.back().o.assign(oPtr,oPtr+oLength);            
            rc = sqlite3_step(loadObservationSequencesStmt);
        }
        if(rc!=SQLITE_DONE) std::cout << "Error when evaluation in LoadObservationSequences()" << std::endl;

        //reset
        sqlite3_reset(loadObservationSequencesStmt);
        sqlite3_clear_bindings(loadObservationSequencesStmt);
    }

    void Database::LoadHMM(HMM& hmm)
    {
        //SQL: "SELECT N M pi A B FROM HMMTable WHERE gestureName==?1 AND quantizerName==?2 AND modelStyle==?3 AND trained==?4;"
        rc = sqlite3_bind_text(loadHMMStmt, 1, hmm.gestureName.c_str(), hmm.gestureName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding gestureName in LoadHMM()" << std::endl;
        rc = sqlite3_bind_text(loadHMMStmt, 2, hmm.quantizerName.c_str(), hmm.quantizerName.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding quantizerName in LoadHMM()" << std::endl;
        rc = sqlite3_bind_text(loadHMMStmt, 3, hmm.modelStyle.c_str(), hmm.modelStyle.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding modelStyle in LoadHMM()" << std::endl;
        rc = sqlite3_bind_int(loadHMMStmt, 4, hmm.trained);
        if(rc!=SQLITE_OK) std::cout << "Error when binding trained in LoadHMM()" << std::endl;

        //evaluation
        rc = sqlite3_step(loadHMMStmt);
        if(rc==SQLITE_ROW){
            hmm.N = sqlite3_column_int(loadHMMStmt, 0);
            hmm.M = sqlite3_column_int(loadHMMStmt, 1);
            const double* piPtr = (const double*)sqlite3_column_blob(loadHMMStmt, 2);
            hmm.pi.assign(piPtr,piPtr+hmm.N);
            const double* APtr = (const double*)sqlite3_column_blob(loadHMMStmt, 3);
            hmm.A.assign(APtr,APtr+hmm.N*hmm.N);
            const double* BPtr = (const double*)sqlite3_column_blob(loadHMMStmt, 4);
            hmm.B.assign(BPtr,BPtr+hmm.N*hmm.M);
        }
        rc = sqlite3_step(loadHMMStmt);
        if(rc!=SQLITE_DONE) std::cout << "Error when evaluation in LoadHMM()" << std::endl;

        //reset
        sqlite3_reset(loadHMMStmt);
        sqlite3_clear_bindings(loadHMMStmt);
    }

    void Database::LoadHMMs(const Quantizer& quantizer, const char* modelStyle, bool trained, vector<HMM>& hmmVec)
    {
        hmmVec.reserve(hmmVecSpace+hmmVec.size());
        //SQL: "SELECT gestureName, N, M, pi, A, B FROM HMMTable WHERE quantizerName==?1 AND modelStyle==?2 AND trained==?3;"
        rc = sqlite3_bind_text(loadHMMsStmt, 1, quantizer.name.c_str(), quantizer.name.size(), SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding quantizerName in LoadHMMs()" << std::endl;
        rc = sqlite3_bind_text(loadHMMsStmt, 2, modelStyle, -1, SQLITE_TRANSIENT);
        if(rc!=SQLITE_OK) std::cout << "Error when binding modelStyle in LoadHMMs()" << std::endl;
        rc = sqlite3_bind_int(loadHMMsStmt, 3, trained);
        if(rc!=SQLITE_OK) std::cout << "Error when binding trained in LoadHMMs()" << std::endl;

        HMM tempHMM(quantizer, modelStyle, trained);

        //evaluation
        rc = sqlite3_step(loadHMMsStmt);
        while(rc==SQLITE_ROW){
            hmmVec.push_back(tempHMM);
            HMM& curHMM = hmmVec.back();
            curHMM.gestureName = (const char*)sqlite3_column_text(loadHMMsStmt, 0);
            curHMM.N = sqlite3_column_int(loadHMMsStmt, 1);
            curHMM.M = sqlite3_column_int(loadHMMsStmt, 2);
            const double* piPtr = (const double*)sqlite3_column_blob(loadHMMsStmt, 3);
            curHMM.pi.assign(piPtr,piPtr+curHMM.N);
            const double* APtr = (const double*)sqlite3_column_blob(loadHMMsStmt, 4);
            curHMM.A.assign(APtr,APtr+curHMM.N*curHMM.N);
            const double* BPtr = (const double*)sqlite3_column_blob(loadHMMsStmt, 5);
            curHMM.B.assign(BPtr,BPtr+curHMM.N*curHMM.M);
            rc = sqlite3_step(loadHMMsStmt);
        }
        if(rc!=SQLITE_DONE) std::cout << "Error when evaluation in LoadHMMs()" << std::endl;
        //reset
        sqlite3_reset(loadHMMsStmt);
        sqlite3_clear_bindings(loadHMMsStmt);
    }

    void Database::DeleteGestures(const char* gestureName){
        string sql = "DELETE FROM GestureTable WHERE gestureName=='" + string(gestureName) + "';";
        char* errMsg = 0;
        rc = sqlite3_exec(db,sql.c_str(),0,0,&errMsg);
        if(rc!=SQLITE_OK) std::cout << errMsg << endl;
    }
    void Database::DeleteObservationSequences(const char* gestureName, const Quantizer& quantizer){
        string sql = "DELETE FROM ObservationSequenceTable WHERE gestureName=='" + string(gestureName)
            + "' AND quantizerName=='" + quantizer.name + "';";
        char* errMsg = 0;
        rc = sqlite3_exec(db,sql.c_str(),0,0,&errMsg);
        if(rc!=SQLITE_OK) std::cout << errMsg << endl;
    }

    Database::Database()
        :dbOpened(false),saveGestureStmt(0),saveObservationSequenceStmt(0),
        isExistHMMStmt(0),updateHMMStmt(0),insertHMMStmt(0),
        loadGesturesStmt(0),loadObservationSequencesStmt(0),loadHMMStmt(0),loadHMMsStmt(0)
    {
        rc = sqlite3_open("WiimoteGR.db", &db);
        if(rc){
            std::cout << "Can't open database: " << sqlite3_errmsg(db) << std::endl;
            sqlite3_close(db);
        }else{
            dbOpened=true;
            rc = sqlite3_exec(db,
                "CREATE TABLE IF NOT EXISTS GestureTable(               "
                "   gestureName TEXT NOT NULL,                          "
                "   dataLength INTEGER NOT NULL,                        "
                "   data BLOB NOT NULL                                  "
                ");                                                     "

                "CREATE TABLE IF NOT EXISTS ObservationSequenceTable(   "
                "   gestureName TEXT NOT NULL,                          "
                "   quantizerName TEXT NOT NULL,                        "
                "   oLength INTEGER NOT NULL,                           "
                "   o BLOB NOT NULL                                     "
                ");                                                     "

                "CREATE TABLE IF NOT EXISTS HMMTable(                   "
                "   gestureName TEXT NOT NULL,                          "
                "   quantizerName TEXT NOT NULL,                        "
                "   modelStyle TEXT NOT NULL,                           "
                "   trained INTEGER NOT NULL,                           "
                "   N INTEGER NOT NULL,                                 "
                "   M INTEGER NOT NULL,                                 "
                "   pi BLOB NOT NULL,                                   "
                "   A BLOB NOT NULL,                                    "
                "   B BLOB NOT NULL                                     "
                ");                                                     ",
                0, 0, &zErrMsg);
            if(rc!=SQLITE_OK) std::cout << "Error when CREATE TABLEs" << std::endl;

            string saveGestureSQL = "INSERT INTO GestureTable VALUES( :gestureName, :dataLength, :data );";
            rc = sqlite3_prepare_v2(db,saveGestureSQL.c_str(),saveGestureSQL.size()+1, &saveGestureStmt, 0);
            if(rc!=SQLITE_OK || saveGestureStmt==0) std::cout << "Error when prepare INSERT INTO GestureTable" << std::endl;

            string saveObservationSequenceSQL = "INSERT INTO ObservationSequenceTable VALUES( :gestureName, :quantizerName, :oLength, :o );";
            rc = sqlite3_prepare_v2(db,saveObservationSequenceSQL.c_str(),saveObservationSequenceSQL.size()+1, &saveObservationSequenceStmt, 0);
            if(rc!=SQLITE_OK || saveObservationSequenceStmt==0) std::cout << "Error when prepare INSERT INTO ObservationSequenceTable" << std::endl;

            string isExistHMMSQL = "SELECT ROWID FROM HMMTable WHERE gestureName==?1 AND quantizerName==?2 AND modelStyle==?3 AND trained==?4 LIMIT 1;";
            rc = sqlite3_prepare_v2(db,isExistHMMSQL.c_str(),isExistHMMSQL.size()+1, &isExistHMMStmt, 0);
            if(rc!=SQLITE_OK || isExistHMMStmt==0) std::cout << "Error when prepare SELECT * FROM HMMTable for test existence" << std::endl;

            string insertHMMSQL = "INSERT INTO HMMTable VALUES( :gestureName, :quantizerName, :modelStyle, :trained, :N, :M, :pi, :A, :B );";
            rc = sqlite3_prepare_v2(db,insertHMMSQL.c_str(),insertHMMSQL.size()+1, &insertHMMStmt, 0);
            if(rc!=SQLITE_OK || insertHMMStmt==0) std::cout << "Error when prepare INSERT INTO HMMTable" << std::endl;

            string updateHMMSQL = "UPDATE HMMTable SET N=?5, M=?6, pi=?7, A=?8, B=?9 "
                "WHERE gestureName==?1 AND quantizerName==?2 AND modelStyle==?3 AND trained==?4;";
            rc = sqlite3_prepare_v2(db,updateHMMSQL.c_str(),updateHMMSQL.size()+1, &updateHMMStmt, 0);
            if(rc!=SQLITE_OK || updateHMMStmt==0) std::cout << "Error when prepare UPDATE HMMTable" << std::endl;

            string loadGesturesSQL = "SELECT dataLength, data FROM GestureTable WHERE gestureName==?1;";
            rc = sqlite3_prepare_v2(db,loadGesturesSQL.c_str(),loadGesturesSQL.size()+1, &loadGesturesStmt, 0);
            if(rc!=SQLITE_OK || loadGesturesStmt==0) std::cout << "Error when prepare loadGesture statement" << std::endl;

            string loadObservationSequencesSQL = "SELECT oLength, o FROM ObservationSequenceTable WHERE gestureName==?1 AND quantizerName==?2;";
            rc = sqlite3_prepare_v2(db,loadObservationSequencesSQL.c_str(),loadObservationSequencesSQL.size()+1, &loadObservationSequencesStmt, 0);
            if(rc!=SQLITE_OK || loadObservationSequencesStmt==0) std::cout << "Error when prepare loadObservationSequences statement" << std::endl;

            string loadHMMSQL = "SELECT N, M, pi, A, B FROM HMMTable WHERE gestureName==?1 AND quantizerName==?2 AND modelStyle==?3 AND trained==?4;";
            rc = sqlite3_prepare_v2(db,loadHMMSQL.c_str(),loadHMMSQL.size()+1, &loadHMMStmt, 0);
            if(rc!=SQLITE_OK || loadHMMStmt==0) std::cout << "Error when prepare loadHMM statement" << std::endl;

            string loadHMMsSQL = "SELECT gestureName, N, M, pi, A, B FROM HMMTable WHERE quantizerName==?1 AND modelStyle==?2 AND trained==?3;";
            rc = sqlite3_prepare_v2(db,loadHMMsSQL.c_str(),loadHMMsSQL.size()+1, &loadHMMsStmt, 0);
            if(rc!=SQLITE_OK || loadHMMsStmt==0) std::cout << "Error when prepare loadHMMs statement" << std::endl;
        }
    }

    Database::~Database()
    {
        sqlite3_close(db);
    }
}
