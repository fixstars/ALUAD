import torch.nn as nn
import torch.nn.init as init
import torchvision.models.squeezenet as sqn

__all__ = ['deep_driving']

class DDAlexNet(nn.Module):

    def __init__(self, output):
        super(DDAlexNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 96, kernel_size=11, stride=4, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2),
            nn.LocalResponseNorm(5, alpha=0.0001, beta=0.75),
            nn.Conv2d(96, 256, kernel_size=5, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2),
            nn.LocalResponseNorm(5, alpha=0.0001, beta=0.75),
            nn.Conv2d(256, 384, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(384, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2),
        )
        self.regression = nn.Sequential(
            nn.Linear(256*5*7, 4096),
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(4096, 4096),
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(4096, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(256, output),
            nn.Sigmoid()
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), 256 * 35)
        x = self.regression(x)
        return x

class DDSqueezeNet(nn.Module):
    def __init__(self, output):
        super(DDSqueezeNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2, ceil_mode=False),
            sqn.Fire(64, 16, 64, 64),
            sqn.Fire(128, 16, 64, 64),
            nn.MaxPool2d(kernel_size=3, stride=2, ceil_mode=False),
            sqn.Fire(128, 32, 128, 128),
            sqn.Fire(256, 32, 128, 128),
            nn.MaxPool2d(kernel_size=3, stride=2, ceil_mode=False),
            sqn.Fire(256, 48, 192, 192),
            sqn.Fire(384, 48, 192, 192),
            sqn.Fire(384, 64, 256, 256),
            sqn.Fire(512, 64, 256, 256),
        )
        # Final convolution is initialized differently form the rest
        final_conv = nn.Conv2d(512, 256, kernel_size=1)
        self.regression1 = nn.Sequential(
            nn.Dropout(p=0.5),
            final_conv,
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )

        self.regression2 = nn.Sequential(
            nn.Linear(256, output),
            nn.Sigmoid(),
        )

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                if m is final_conv:
                    init.normal_(m.weight, mean=0.0, std=0.01)
                else:
                    init.kaiming_uniform_(m.weight)
                if m.bias is not None:
                    init.constant_(m.bias, 0)

    def forward(self, x):
        x = self.features(x)
        x = self.regression1(x)
        x = x.view(x.size(0), 256)
        x = self.regression2(x)
        return x



def deep_driving(idx,output,**kwargs):
    r"""DeepDirving model
    """
    #model = DDAlexNet(**kwargs)
    if idx == 'alex':
        model = DDAlexNet(output,**kwargs)
    elif idx == 'squeeze':
        model = DDSqueezeNet(output,**kwargs)
    return model
