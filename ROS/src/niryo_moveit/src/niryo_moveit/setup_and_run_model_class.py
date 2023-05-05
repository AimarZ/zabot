import torch
import torch.nn as nn
import torchvision

import numpy as np

from PIL import Image



def preload():
    '''pre-load VGG model weights, for transfer learning. Automatically cached for later use.'''
    torchvision.models.vgg16(pretrained=True)


class PoseEstimationNetwork(torch.nn.Module):
    """
    PoseEstimationNetwork: Neural network based on the VGG16 neural network
    architecture developed by Tobin at al. (https://arxiv.org/pdf/1703.06907.pdf). 
    The model is a little bit different from the original one
    but we still import the model as it has already been trained on a huge
    dataset (ImageNet) and even if we change a bit its architecture, the main
    body of it is unchanged and the weights of the final model will not be too
    far from the original one. We call this method "transfer learning".
    The network is composed by two branches: one for the translation
    (prediction of a 3 dimensional vector corresponding to x, y, z coordinates) and
    one for the orientation (prediction of a 4 dimensional vector corresponding to
    a quaternion)
    """

    def __init__(self, *, is_symetric):
        torch.autograd.set_detect_anomaly(True)
        super(PoseEstimationNetwork, self).__init__()
        self.is_symetric = is_symetric
        self.model_backboneRGB = torchvision.models.vgg16(pretrained=True) # uses cache
        self.model_backboneDepth = torchvision.models.vgg16(pretrained=True) # uses cache
        # remove the original classifier
        self.model_backboneRGB.classifier = torch.nn.Identity()
        self.model_backboneDepth.classifier = torch.nn.Identity()
        self.num_cubes = 1
        self.num_classes = 4
        
        self.depth_preprocess = torch.nn.Conv2d(1, 3, kernel_size=5, stride=1, padding=2)

        self.class_block = torch.nn.Sequential(
            torch.nn.Linear(25088*2, 256*2),
            torch.nn.BatchNorm1d(256*2),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(256*2, 64*2),
            torch.nn.BatchNorm1d(64*2),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(64*2, self.num_classes),
        )


        self.depth_preprocess.weight = torch.nn.Parameter(torch.FloatTensor([[[
              [1, 4, 7,  4, 1],
              [4, 16, 26, 16, 4],
              [7, 26, 41, 26, 4],
              [4, 16, 26, 16, 4],
              [1, 4, 7,  4, 1]]],


            [[[0, 0, 0, 0, 0],
              [0, 16, 26, 16, 0],
              [0, 26, 41, 26, 0],
              [0, 16, 26, 16, 0],
              [0, 0, 0, 0, 0]]],


            [[[0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0]]]]))
        self.depth_preprocess.weight.requires_grad = False

    def forward(self, x_rgb, x_depth):
        #channel1 = normalize(Depth_image) == /255 or whatever
        #channel2 = 
        x_rgb = self.model_backboneRGB(x_rgb)
        x_depth = self.depth_preprocess(x_depth)
        x_depth = self.model_backboneDepth(x_depth)
        x = torch.cat((x_rgb, x_depth), dim=1)

        output_class = self.class_block(x)

        return output_class


class LinearNormalized(torch.nn.Module):
    """
    Custom activation function which normalizes the input.
    It will be used to normalized the output of the orientation
    branch in our model because a quaternion vector is a
    normalized vector
    """

    def __init__(self, num_cubes):
        super(LinearNormalized, self).__init__()
        self.num_cubes = num_cubes

    def forward(self, x):
        return self._linear_normalized(x)

    def _linear_normalized(self, x):
        """
        Activation function which normalizes an input
        It will be used in the orientation network because
        a quaternion is a normalized vector.
        Args:
            x (pytorch tensor with shape (batch_size, 4*num_cubes)): the input of the model
        Returns:
            a pytorch tensor normalized vector with shape(batch_size, 4*num_cubes)
        """
        for i in range(self.num_cubes):
          norm = torch.norm(x[:,i*4:4*(i+1)], p=2, dim=1).unsqueeze(0)
          for index in range(norm.shape[1]):
              if norm[0, index].item() == 0.0:
                  norm[0, index] = 1.0
          y = torch.transpose(x[:,i*4:4*(i+1)], 0, 1)
          y = torch.div(y, norm)
          y = torch.transpose(y, 0, 1)
          if i==0:
            aux = y
          else:
            aux = torch.cat((aux,y),dim=1)
        x = aux
        return x






def pre_process_image(path_image, device):
    
    image_RGB = Image.open(path_image).convert("RGB")
    transform = get_transform()
    imageRGB = [transform(image_RGB).unsqueeze(0)]
    imageRGB = list(img.to(device) for img in imageRGB)
    
    imageDepth = Image.open(path_image).convert("RGBA").getchannel("A")
    imageDepth = [transform(imageDepth).unsqueeze(0)]
    imageDepth = list(img.to(device) for img in imageDepth)

    return imageRGB, imageDepth

def get_transform():
    """
    Apply a transform on the input image tensor
    Returns:
        https://pytorch.org/docs/stable/torchvision/transforms.html
    """
    transform = torchvision.transforms.Compose(
        [
            torchvision.transforms.Resize(
                (
                    224,
                    224,
                )
            ),
            torchvision.transforms.ToTensor(),
        ]
    )
    return transform

global model
model = None

def run_model_main_class(image_file_png, model_file_name):
    global model

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    if model is None:
        checkpoint = torch.load(model_file_name, map_location=device)
        model = PoseEstimationNetwork(is_symetric=False)
        model.load_state_dict(checkpoint["model"])
        model.to(device)
        model.eval()

    imageRGB, imageDepth = pre_process_image(image_file_png, device)
    output_class = model(torch.stack(imageRGB).reshape(-1, 3, 224, 224).to(device), torch.stack(imageDepth).reshape(-1, 1, 224, 224).to(device))
    output_class = torch.argmax(torch.nn.functional.softmax(output_class, dim=1))
    return output_class
