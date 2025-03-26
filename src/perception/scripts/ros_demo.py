#!/home/rslim/miniconda3/envs/pyenv/bin/python3

"""visualize results of the pre-trained model """

# Example
# python demo.py --net res101 --dataset vg --load_dir models --cuda
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import os
import sys
print(os.getcwd())
# import FasterRCNN._init_paths
# from abc import _init_paths
import FasterRCNN._init_paths
# sys.path.insert(0, '../FasterRCNN')
# print("sys.path ",sys.path)

from FasterRCNN import _init_paths

# import _init_paths
import numpy as np
import argparse
import pprint
import pdb
import time
import cv2
import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.optim as optim

import torchvision.transforms as transforms
import torchvision.datasets as dset
# from scipy.misc import imread
from imageio import imread
from roi_data_layer.roidb import combined_roidb
from roi_data_layer.roibatchLoader import roibatchLoader
from model.utils.config import cfg, cfg_from_file, cfg_from_list, get_output_dir
from model.rpn.bbox_transform import clip_boxes
# from model.nms.nms_wrapper import nms
# from model.roi_layers import nms
from torchvision.ops import nms
from model.rpn.bbox_transform import bbox_transform_inv
from model.utils.net_utils import save_net, load_net, vis_detections
from model.utils.blob import im_list_to_blob
from model.faster_rcnn.vgg16 import vgg16
from model.faster_rcnn.resnet import resnet
import pdb

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvas
from matplotlib.figure import Figure

import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from custom_msgs.msg import Bbox, Bboxes
from custom_msgs.srv import Detect, DetectResponse
from std_srvs.srv import SetBool, SetBoolResponse

from message_filters import ApproximateTimeSynchronizer, Subscriber
from visualization_msgs.msg import *

if torch.cuda.is_available():
    device = torch.device("cuda")
    print('There are %d GPU(s) available.' %torch.cuda.device_count())
    print('We will use the GPU:',torch.cuda.get_device_name(0))
else:
    print('No GPU available, using the CPU instead')
    device = torch.device("cpu")

try:
    xrange          # Python 2
except NameError:
    xrange = range  # Python 3

import os

class FasterRCNN_ros():
  def __init__(self):
    self.scan_flag = True
    self.output_pub = rospy.Publisher("/faster_rcnn_output", Image, queue_size=1)

    #  Set buff_size to 2^24 bytes for image, else result in delay
    # self.image_sub  = Subscriber("/camera/color/image_raw", Image, self.imCallback, queue_size = 1, buff_size = 2**24)
    self.bridge     = CvBridge()
    self.args = parse_args()
    # Door, doorway ... = 315, 55, 946, 997, 1040, 1391, 1490
    # Table = 107
    self.accepted_cls = [315, 55, 946, 997, 1040, 1391, 1490, 107]
    print('Called with args:')
    print("in ros_demo.py ", self.args)

    # set cfg according to the dataset used to train the pre-trained model
    if self.args.dataset == "pascal_voc":
        self.args.set_cfgs = ['ANCHOR_SCALES', '[8, 16, 32]', 'ANCHOR_RATIOS', '[0.5,1,2]']
    elif self.args.dataset == "pascal_voc_0712":
        self.args.set_cfgs = ['ANCHOR_SCALES', '[8, 16, 32]', 'ANCHOR_RATIOS', '[0.5,1,2]']
    elif self.args.dataset == "coco":
        self.args.set_cfgs = ['ANCHOR_SCALES', '[4, 8, 16, 32]', 'ANCHOR_RATIOS', '[0.5,1,2]']
    elif self.args.dataset == "imagenet":
        self.args.set_cfgs = ['ANCHOR_SCALES', '[8, 16, 32]', 'ANCHOR_RATIOS', '[0.5,1,2]']
    elif self.args.dataset == "vg":
        self.args.set_cfgs = ['ANCHOR_SCALES', '[4, 8, 16, 32]', 'ANCHOR_RATIOS', '[0.5,1,2]']

    self.args.cfg_file = os.path.dirname(os.path.realpath(__file__)) + "/" + self.args.cfg_file
    self.args.load_dir = os.path.dirname(os.path.realpath(__file__)) + "/" + self.args.load_dir
    self.args.classes_dir = os.path.dirname(os.path.realpath(__file__)) + "/" + self.args.classes_dir
    if self.args.cfg_file is not None:
        cfg_from_file(self.args.cfg_file)
    if self.args.set_cfgs is not None:
        cfg_from_list(self.args.set_cfgs)
    
    # self.cuda
    if device.type != 'cpu':
        self.args.cuda = True
    else:
        self.args.cuda = False

    cfg.USE_GPU_NMS = self.args.cuda

    print('Using config:')
    pprint.pprint(cfg)
    np.random.seed(cfg.RNG_SEED)

    # Load faster rcnn model
    if not os.path.exists(self.args.load_dir):
        raise Exception('There is no input directory for loading network from ' + self.args.load_dir)
    load_name = os.path.join(self.args.load_dir, 'faster_rcnn_{}_{}.pth'.format(self.args.net, self.args.dataset))

    # Load classes
    self.classes = ['__background__']
    with open(os.path.join(self.args.classes_dir, 'objects_vocab.txt')) as f:
        for object in f.readlines():
            self.classes.append(object.split(',')[0].lower().strip())

    # initilize the network here.
    if self.args.net == 'res101':
        self.fasterRCNN = resnet(self.classes, 101, pretrained=False, class_agnostic= self.args.class_agnostic)
    else:
        print("network is not defined")
        pdb.set_trace()

    # Build faster rcnn
    self.fasterRCNN.create_architecture()

    print("load checkpoint %s" % (load_name))
    if self.args.cuda > 0:
        checkpoint = torch.load(load_name)
    else:
        checkpoint = torch.load(load_name, map_location=(lambda storage, loc: storage))
    self.fasterRCNN.load_state_dict(checkpoint['model'])
    if 'pooling_mode' in checkpoint.keys():
        cfg.POOLING_MODE = checkpoint['pooling_mode']

    print('load model successfully!')

    if self.args.cuda > 0:
        cfg.CUDA = True

    if self.args.cuda > 0:
        self.fasterRCNN.cuda()

    self.fasterRCNN.eval()

    # max_per_image = 100
    self.thresh = 0.05
    self.vis = True
    torch.backends.cudnn.benchmark = True
    self.RCNN_service = rospy.Service("faster_rcnn_srv", Detect, self.serviceCB)

  def serviceCB(self, faster_srv):
    res = DetectResponse()
    res.bboxes = self.forward(faster_srv.im_req)
    return res

  def forward(self, data):
    print("forward")
    start_time = time.time()
    bboxes_msg = Bboxes()
    # bboxes_msg.header = data.header

    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:       
        print(e)
        return bboxes_msg
    #im = cv2.resize(cv_image, (600,850), interpolation=cv2.INTER_LINEAR)
    im_in = np.array(cv_image)
    if len(im_in.shape) == 2:
        im_in = im_in[:,:,np.newaxis]
        im_in = np.concatenate((im_in,im_in,im_in), axis=2)
    # rgb -> bgr
    im = im_in[:,:,::-1]

    blobs, im_scales = _get_image_blob(im)
    assert len(im_scales) == 1, "Only single-image batch implemented"
    im_blob = blobs
    im_info_np = np.array([[im_blob.shape[1], im_blob.shape[2], im_scales[0]]], dtype=np.float32)

    # initilize the tensor holder here.
    im_data = torch.FloatTensor(1)
    im_info = torch.FloatTensor(1)
    num_boxes = torch.LongTensor(1)
    gt_boxes = torch.FloatTensor(1)

    # ship to cuda
    if self.args.cuda > 0:
        im_data = im_data.cuda()
        im_info = im_info.cuda()
        num_boxes = num_boxes.cuda()
        gt_boxes = gt_boxes.cuda()

    # make variable
    with torch.no_grad():
        im_data = Variable(im_data)
        im_info = Variable(im_info)
        num_boxes = Variable(num_boxes)
        gt_boxes = Variable(gt_boxes)

    im_data_pt = torch.from_numpy(im_blob)
    im_data_pt = im_data_pt.permute(0, 3, 1, 2)
    im_info_pt = torch.from_numpy(im_info_np)

    with torch.no_grad():
            im_data.resize_(im_data_pt.size()).copy_(im_data_pt)
            im_info.resize_(im_info_pt.size()).copy_(im_info_pt)
            gt_boxes.resize_(1, 1, 5).zero_()
            num_boxes.resize_(1).zero_()
    # pdb.set_trace()
    det_tic = time.time()
    print("preprocess time:", det_tic-start_time)

    rois, cls_prob, bbox_pred, \
    rpn_loss_cls, rpn_loss_box, \
    RCNN_loss_cls, RCNN_loss_bbox, \
    rois_label = self.fasterRCNN(im_data, im_info, gt_boxes, num_boxes)
    det_toc = time.time()
    print("detect time:", det_toc-det_tic)

    scores = cls_prob.data
    boxes = rois.data[:, :, 1:5]
    # regression_tic = time.time()

    # print("conversion time:", regression_tic - det_toc)
    if cfg.TEST.BBOX_REG:
        # Apply bounding-box regression deltas
        box_deltas = bbox_pred.data
        if cfg.TRAIN.BBOX_NORMALIZE_TARGETS_PRECOMPUTED:
        # Optionally normalize targets by a precomputed mean and stdev
            if self.args.class_agnostic:
                if self.args.cuda > 0:
                    box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS).cuda() \
                            + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS).cuda()
                else:
                    box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS) \
                            + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS)
            else:
                if self.args.cuda > 0:
                    box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS).cuda() \
                            + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS).cuda()
                else:
                    box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS) \
                            + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS)
                box_deltas = box_deltas.view(1, -1, 4 * len(self.classes))

        pred_boxes = bbox_transform_inv(boxes, box_deltas, 1)
        pred_boxes = clip_boxes(pred_boxes, im_info.data, 1)
    else:
        # Simply repeat the boxes, once for each class
        pred_boxes = np.tile(boxes, (1, scores.shape[1]))

    pred_boxes /= im_scales[0]

    scores = scores.squeeze()
    pred_boxes = pred_boxes.squeeze()
    # print(det_toc-regression_tic)
    # plot_tic = time.time()

    max_conf = torch.zeros((pred_boxes.shape[0]))
    if self.args.cuda > 0:
        max_conf = max_conf.cuda()
    if self.vis:
        im2show = np.copy(im)

    #1, len(self.classes)
    # 314,316
    for j in xrange(1, len(self.classes)):
        inds = torch.nonzero(scores[:,j]>self.thresh).view(-1)
        # if there is det
        if inds.numel() > 0:
            cls_scores = scores[:,j][inds]
            _, order = torch.sort(cls_scores, 0, True)
            if self.args.class_agnostic:
                cls_boxes = pred_boxes[inds, :]
            else:
                cls_boxes = pred_boxes[inds][:, j * 4:(j + 1) * 4]

            cls_dets = torch.cat((cls_boxes, cls_scores.unsqueeze(1)), 1)
            # cls_dets = torch.cat((cls_boxes, cls_scores), 1)
            cls_dets = cls_dets[order]
            # keep = nms(cls_dets, cfg.TEST.NMS, force_cpu=not cfg.USE_GPU_NMS)
            keep = nms(cls_boxes[order, :], cls_scores[order], cfg.TEST.NMS)
            cls_dets = cls_dets[keep.view(-1).long()]
            index = inds[order[keep]]
            max_conf[index] = torch.where(scores[index, j] > max_conf[index], scores[index, j], max_conf[index])
            if self.vis:
                im2show = vis_detections(im2show, self.classes[j], cls_dets.cpu().numpy(), 0.5)
    # print("class for loop time", time.time()-plot_tic)

    if self.args.cuda > 0:
        keep_boxes = torch.where(max_conf >= conf_thresh, max_conf, torch.tensor(0.0).cuda())
    else:
        keep_boxes = torch.where(max_conf >= conf_thresh, max_conf, torch.tensor(0.0))
    keep_boxes = torch.squeeze(torch.nonzero(keep_boxes))
    # print(keep_boxes.nelement())
    try:
        if len(keep_boxes) < MIN_BOXES:
            keep_boxes = torch.argsort(max_conf, descending = True)[:MIN_BOXES]
        if len(keep_boxes) > MAX_BOXES:
            keep_boxes = torch.argsort(max_conf, descending = True)[:MAX_BOXES]
    except:
        return bboxes_msg

    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    if(len(keep_boxes) == 0):
        print("No object detected")
        return bboxes_msg

    boxes = pred_boxes[keep_boxes]
    objects = torch.argmax(scores[keep_boxes][:,1:], dim=1)
    
    for i in range(len(keep_boxes)):
        bbox_msg = Bbox()
        kind = objects[i]+1
        bbox = boxes[i, kind * 4: (kind + 1) * 4]
        # bbox = boxes[i]
        if bbox[0] == 0:
            bbox[0] = 1
        if bbox[1] == 0:
            bbox[1] = 1
        if(not self.check_within_list(objects[i]+1)):
            continue
        cls = self.classes[objects[i]+1]
        cv2.rectangle(im, (int(bbox[0]),int(bbox[1])), (int(bbox[2]),int(bbox[3])), (255,0,0), 2)
        cv2.putText(im, cls, (int(bbox[0]), int(bbox[1])), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2)
        
        bbox_msg.top_left, bbox_msg.btm_left, bbox_msg.top_right, bbox_msg.btm_right  = self.save_pixel(bbox)
        bbox_msg.camera_name = data.header.frame_id
        bbox_msg.cls = cls
        print("bbox_msg :",bbox_msg)
        bboxes_msg.bboxes.append(bbox_msg)

    # print("plot time", time.time() - plot_tic)
    
    output_msg = self.bridge.cv2_to_imgmsg(im, "bgr8")
    # output_msg.header = data.header

    self.output_pub.publish(output_msg)
    end_time = time.time()
    rospy.loginfo("Time spent for faster-rcnn detection callback %f sec", (end_time - start_time))
    # print("callback")
    print("bboxes_msg :", bboxes_msg)
    return bboxes_msg

  def check_within_list(self, input_cls):
      for i in self.accepted_cls:
        if(input_cls == i):
          return True
      return False
    

  def save_pixel(self, bbox):
    x1 = int(bbox[0].item())
    y1 = int(bbox[1].item())
    x2 = int(bbox[2].item())
    y2 = int(bbox[3].item())

    top_left = Point()
    btm_left = Point()
    top_right = Point()
    btm_right = Point()
    
    top_left.x = x1
    top_left.y = y1

    btm_left.x = x1
    btm_left.y = y2

    top_right.x = x2
    top_right.y = y1

    btm_right.x = x2
    btm_right.y = y2

    return top_left, btm_left, top_right, btm_right
    

def parse_args():
  """
  Parse input arguments
  """
  parser = argparse.ArgumentParser(description='Train a Fast R-CNN network')
  parser.add_argument('--dataset', dest='dataset',
                      help='training dataset',
                      default='vg', type=str)
  parser.add_argument('--cfg', dest='cfg_file',
                      help='optional config file',
                      default='FasterRCNN/cfgs/res101.yml', type=str)
  parser.add_argument('--net', dest='net',
                      help='vgg16, res50, res101, res152',
                      default='res101', type=str)
  parser.add_argument('--load_dir', dest='load_dir',
                      help='directory to load models',
                      default="FasterRCNN/models")
  parser.add_argument('--image_dir', dest='image_dir',
                      help='directory to load images for demo',
                      default="images")
  parser.add_argument('--image_file', dest='image_file',
                      help='the file name of load images for demo',
                      default="img1.jpg")
  parser.add_argument('--classes_dir', dest='classes_dir',
                      help='directory to load object classes for classification',
                      default="FasterRCNN/data/genome/1600-400-20")
#   parser.add_argument('--cuda', dest='cuda',
#                       help='whether use CUDA',
#                       action='store_true')
  parser.add_argument('--mGPUs', dest='mGPUs',
                      help='whether use multiple GPUs',
                      action='store_true')
  parser.add_argument('--set', dest='set_cfgs',
                      help='set config keys', default=None,
                      nargs=argparse.REMAINDER)
  parser.add_argument('--cag', dest='class_agnostic',
                      help='whether perform class_agnostic bbox regression',
                      action='store_true')
  parser.add_argument('--parallel_type', dest='parallel_type',
                      help='which part of model to parallel, 0: all, 1: model before roi pooling',
                      default=0, type=int)
  parser.add_argument('--vis', dest='vis',
                      help='visualization mode',
                      action='store_true')

  args, unknown = parser.parse_known_args()
  return args

lr = cfg.TRAIN.LEARNING_RATE
momentum = cfg.TRAIN.MOMENTUM
weight_decay = cfg.TRAIN.WEIGHT_DECAY
conf_thresh = 0.4
MIN_BOXES = 10
MAX_BOXES = 36

def _get_image_blob(im):
  """Converts an image into a network input.
  Arguments:
    im (ndarray): a color image in BGR order
  Returns:
    blob (ndarray): a data blob holding an image pyramid
    im_scale_factors (list): list of image scales (relative to im) used
      in the image pyramid
  """
  im_orig = im.astype(np.float32, copy=True)
  im_orig -= cfg.PIXEL_MEANS

  im_shape = im_orig.shape
  im_size_min = np.min(im_shape[0:2])
  im_size_max = np.max(im_shape[0:2])

  processed_ims = []
  im_scale_factors = []

  for target_size in cfg.TEST.SCALES:
      im_scale = float(target_size) / float(im_size_min)
      # Prevent the biggest axis from being more than MAX_SIZE
      if np.round(im_scale * im_size_max) > cfg.TEST.MAX_SIZE:
        im_scale = float(cfg.TEST.MAX_SIZE) / float(im_size_max)
      im = cv2.resize(im_orig, None, None, fx=im_scale, fy=im_scale,
              interpolation=cv2.INTER_LINEAR)
      im_scale_factors.append(im_scale)
      processed_ims.append(im)

  # Create a blob to hold the input images
  blob = im_list_to_blob(processed_ims)

  return blob, np.array(im_scale_factors)



if __name__ == '__main__':

  rospy.init_node('FasterRCNN_node')
  fasterRCNN_obj = FasterRCNN_ros()

  rospy.spin()
