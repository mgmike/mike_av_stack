#! ~/anaconda3/envs/waymo/bin/python

import numpy as np
import torch
import time
from easydict import EasyDict as edict

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# model-related
from tools.objdet_models.resnet.models import fpn_resnet
from tools.objdet_models.resnet.utils.evaluation_utils import decode, post_processing 
from tools.objdet_models.resnet.utils.torch_utils import _sigmoid


# load model-related parameters into an edict
def load_configs_model(model_name='darknet', configs=None):

    # init config file, if none has been passed
    if configs==None:
        configs = edict()  

    # get parent directory of this file to enable relative paths
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = configs.model_path = os.path.abspath(os.path.join(curr_path, os.pardir))    
    
    # set parameters according to model type
    if model_name == "darknet":
        configs.model_path = os.path.join(parent_path, 'tools', 'objdet_models', 'darknet')
        configs.pretrained_filename = os.path.join(configs.model_path, 'pretrained', 'complex_yolov4_mse_loss.pth')
        configs.arch = 'darknet'
        configs.batch_size = 4
        configs.cfgfile = os.path.join(configs.model_path, 'config', 'complex_yolov4.cfg')
        configs.conf_thresh = 0.5
        configs.num_classes = 3
        configs.distributed = False
        configs.img_size = 608
        configs.nms_thresh = 0.4
        configs.num_samples = None
        configs.num_workers = 4
        configs.pin_memory = True
        configs.use_giou_loss = False

    elif model_name == 'fpn_resnet':
        ####### ID_S3_EX1-3 START #######     
        #######
        print("student task ID_S3_EX1-3")
        configs.model_path = os.path.join(parent_path, 'tools', 'objdet_models', 'resnet')
        configs.pretrained_filename = os.path.join(configs.model_path, 'pretrained', 'fpn_resnet_18_epoch_300.pth')
        configs.imagenet_pretrained = True
        configs.arch = 'fpn_resnet'
        configs.num_layers = 18
        configs.batch_size = 4
        configs.K = 50
        # configs.cfgfile = os.path.join(configs.model_path, 'config', 'complex_yolov4.cfg')
        configs.conf_thresh = 0.5
        configs.peak_thresh = 0.2
        configs.distributed = False
        configs.img_size = 608
        configs.nms_thresh = 0.4
        configs.num_samples = None
        configs.num_workers = 1
        configs.pin_memory = True

        configs.input_size = (608, 608)
        configs.hm_size = (152, 152)
        configs.down_ratio = 4
        configs.max_objects = 50

        configs.imagenet_pretrained = False
        configs.head_conv = 64
        configs.num_classes = 3
        configs.num_center_offset = 2
        configs.num_z = 1
        configs.num_dim = 3
        configs.num_direction = 2  # sin, cos

        configs.heads = {
            'hm_cen': configs.num_classes,
            'cen_offset': configs.num_center_offset,
            'direction': configs.num_direction,
            'z_coor': configs.num_z,
            'dim': configs.num_dim
        }
        configs.num_input_features = 4

        #######
        ####### ID_S3_EX1-3 END #######     

    else:
        raise ValueError("Error: Invalid model name")

    # GPU vs. CPU
    configs.no_cuda = True # if true, cuda is not used
    configs.gpu_idx = 0  # GPU index to use.
    configs.device = torch.device('cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx))
    
    # Evaluation params
    configs.min_iou = 0.5

    return configs

# load all object-detection parameters into an edict
def load_configs(model_name='fpn_resnet', configs=None):

    # init config file, if none has been passed
    if configs==None:
        configs = edict()    

    # birds-eye view (bev) parameters
    configs.lim_x = [0, 50] # detection range in m
    configs.lim_y = [-25, 25]
    configs.lim_z = [-1, 3]
    configs.lim_r = [0, 1.0] # reflected lidar intensity
    configs.bev_width = 608  # pixel resolution of bev image
    configs.bev_height = 608 

    # add model-dependent parameters
    configs = load_configs_model(model_name, configs)

    # visualization parameters
    configs.output_width = 608 # width of result image (height may vary)
    configs.obj_colors = [[0, 255, 255], [0, 0, 255], [255, 0, 0]] # 'Pedestrian': 0, 'Car': 1, 'Cyclist': 2

    return configs

def time_synchronized():
    torch.cuda.synchronize() if torch.cuda.is_available() else None
    return time.time()

def extract_3d_bb(det, configs, cls_id = 1):
    ## step 3 : perform the conversion using the limits for x, y and z set in the configs structure
    # (scores-0:1, x-1:2, y-2:3, z-3:4, dim-4:7, yaw-7:8)
    _score, _x, _y, _z, _h, _w, _l, _yaw = det
    _yaw = -_yaw
    bound_size_x = configs.lim_x[1] - configs.lim_x[0]
    bound_size_y = configs.lim_y[1] - configs.lim_y[0]
    x = _y / configs.bev_height * bound_size_x + configs.lim_x[0]
    y = _x / configs.bev_width * bound_size_y + configs.lim_y[0]
    z = _z + configs.lim_z[0]
    w = _w / configs.bev_width * bound_size_y
    l = _l / configs.bev_height * bound_size_x
    if x < configs.lim_x[0]  or x > configs.lim_x[1] or y < configs.lim_y[0] or y > configs.lim_y[1]:
        return []
    else:
        return [cls_id, x, y, z, _h, w, l, _yaw]

# detect trained objects in birds-eye view
def detect_objects(input_bev_maps, model, configs):

    
    # Extract 3d bounding boxes from model response
    print("student task ID_S3_EX2")
    objects = [] 

    # deactivate autograd engine during test to reduce memory usage and speed up computations
    with torch.no_grad():  

        # perform inference

        # decode model output into target object format
        # if 'darknet' in configs.arch:

        #     # perform post-processing
        #     outputs = model(input_bev_maps)
        #     output_post = post_processing_v2(outputs, conf_thresh=configs.conf_thresh, nms_thresh=configs.nms_thresh) 
        #     detections = []
        #     for sample_i in range(len(output_post)):
        #         if output_post[sample_i] is None:
        #             continue
        #         detection = output_post[sample_i]
        #         for obj in detection:
        #             x, y, w, l, im, re, _, _, _ = obj
        #             yaw = np.arctan2(im, re)
        #             detections.append([1, x, y, 0.0, 1.50, w, l, yaw])    

        #     for det in detections:
        #         obj = extract_3d_bb(det, configs)
        #         if len(obj) > 0:
        #             objects.append(obj)

        if 'fpn_resnet' in configs.arch:
            # decode output and perform post-processing
            
            ####### ID_S3_EX1-5 START #######     
            #######
            print("student task ID_S3_EX1-5")

            # perform post-processing

            t1 = time_synchronized()
            outputs = model(input_bev_maps)
            outputs['hm_cen'] = _sigmoid(outputs['hm_cen'])
            outputs['cen_offset'] = _sigmoid(outputs['cen_offset'])
            # detections size (batch_size, K, 10)
            detections = decode(outputs['hm_cen'], outputs['cen_offset'], outputs['direction'], outputs['z_coor'],
                                outputs['dim'], K=configs.K)
            detections = detections.cpu().numpy().astype(np.float32)
            detections = post_processing(detections, configs)
            t2 = time_synchronized() 

            detections = detections[0]
            #######
            ####### ID_S3_EX1-5 END #######     

            ####### ID_S3_EX2 START #######     
            #######
            
            ## detections contains an array of length 3 where index 0 pertains to a list of pidestrians, 
            ## index 1 is a list of vehicles and index 2 is a list of cyclests. 
            ## Each array can contain a list of detections

            ## step 2 : loop over all detections
            for cls_id in range(configs.num_classes):
                ## step 1 : check whether there are any detections
                if len(detections[cls_id]) > 0:
                    for det in detections[cls_id]:
                        ## step 4 : append the current object to the 'objects' array
                        obj = extract_3d_bb(det, configs)
                        if len(obj) > 0:
                            objects.append(obj)
 
    
    return objects    

