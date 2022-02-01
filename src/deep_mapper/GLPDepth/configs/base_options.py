import argparse


class BaseOptions():
    def __init__(self):
        pass

    def initialize(self):
        parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        # base configs
        parser.add_argument('--gpu_or_cpu',   type=str, default='cpu')
        parser.add_argument('--data_path',    type=str, default='./input')
        parser.add_argument('--dataset',      type=str, default='kitti',
                            choices=['nyudepthv2', 'kitti', 'imagepath'])
        parser.add_argument('--exp_name',     type=str, default='test')

        parser.add_argument('--batch_size',   type=int, default=1)
        parser.add_argument('--workers',      type=int, default=1)
        
        # depth configs
        parser.add_argument('--max_depth',      type=float, default=80.0)
        parser.add_argument('--max_depth_eval', type=float, default=80.0)
        parser.add_argument('--min_depth_eval', type=float, default=1e-3)        
        parser.add_argument('--do_kb_crop',     type=int, default=1)

        parser.add_argument('--kitti_crop', type=str, default=None,
                            choices=['garg_crop', 'eigen_crop'])

        return parser
