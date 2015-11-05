#include "Component/ModelComponent.H"
#include "Component/ModelManager.H"
#include "Learn/SVMClassifier.H"
#include "svm.h"


int main(int argc, char **argv)
{
    MYLOGVERB = LOG_INFO;

  ModelManager *mgr = new ModelManager("Test Hmax with Feature Learning");


  mgr->exportOptions(MC_RECURSE);

  // required arguments
  // <c1patchesDir> <dir|list> <id> <outputfile>
  //
  // <id> is the given id for the given set of images
  // --in only needs to happen if we are loading the patches

  if (mgr->parseCommandLine(
                            (const int)argc, (const char**)argv, "<features> <svmRange> <scaledoutput>", 3,3) == false)
    return 1;

  std::string features,svmRange,output;
  features= mgr->getExtraArg(0);
  svmRange = mgr->getExtraArg(1);
  output = mgr->getExtraArg(2);


  SVMClassifier svm;
  svm.readRange(svmRange);
  svm.rescale(features.c_str(),output.c_str());
}
