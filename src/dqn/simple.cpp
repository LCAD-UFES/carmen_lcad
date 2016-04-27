#include <caffe/caffe.hpp>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <boost/smart_ptr.hpp>
#include "dqn_caffe.h"
#include <cstdio>

DqnCaffe *dqn;

int
main(int argc, char **argv)
{
//	google::InitGoogleLogging(argv[0]);
//	google::InstallFailureSignalHandler();
//	google::LogToStderr();
//
//	caffe::Caffe::SetDevice(0);
//	caffe::Caffe::set_mode(caffe::Caffe::GPU);
//
//	boost::shared_ptr<caffe::Solver<float> > solver;
//
//	caffe::SolverParameter solver_param;
//	caffe::ReadProtoFromTextFileOrDie("dqn_solver.prototxt", &solver_param);
//	solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));

	dqn = new DqnCaffe(DqnParams(), argv[0]);
	dqn->Initialize();

	printf("OK\n");
	return 0;
}
