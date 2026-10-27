"""Minimal TwinLiteNetPlus runtime with an optional lane-line branch.

The module names intentionally match the upstream checkpoint. Training, dataset
and evaluation code are excluded from the runtime package.
"""

import torch
import torch.nn as nn
import torch.nn.functional as functional


CHANNEL_CONFIGS = {
    "nano": {"p": 1, "q": 1, "channels": [4, 8, 16, 32, 64]},
    "small": {"p": 2, "q": 3, "channels": [8, 16, 32, 64, 128]},
    "medium": {"p": 3, "q": 5, "channels": [16, 32, 64, 128, 256]},
    "large": {"p": 5, "q": 7, "channels": [32, 64, 128, 256, 512]},
}


def patch_split(tensor, bin_size):
    batch, channels, height, width = tensor.size()
    bins_h, bins_w = bin_size
    region_h = height // bins_h
    region_w = width // bins_w
    output = tensor.view(batch, channels, bins_h, region_h, bins_w, region_w)
    output = output.permute(0, 2, 4, 3, 5, 1).contiguous()
    return output.view(batch, -1, region_h, region_w, channels)


def patch_recover(tensor, bin_size):
    batch, _, region_h, region_w, channels = tensor.size()
    bins_h, bins_w = bin_size
    output = tensor.view(batch, bins_h, bins_w, region_h, region_w, channels)
    output = output.permute(0, 5, 1, 3, 2, 4).contiguous()
    return output.view(batch, channels, region_h * bins_h, region_w * bins_w)


class GCN(nn.Module):
    def __init__(self, num_node, num_channel):
        super().__init__()
        self.conv1 = nn.Conv2d(num_node, num_node, kernel_size=1, bias=False)
        self.relu = nn.PReLU(num_node)
        self.conv2 = nn.Linear(num_channel, num_channel, bias=False)

    def forward(self, tensor):
        output = self.conv1(tensor)
        output = self.relu(output + tensor)
        return self.conv2(output)


class CAAM(nn.Module):
    """Class Activation Attention Module."""

    def __init__(self, feat_in, num_classes, bin_size, norm_layer):
        super().__init__()
        feat_inner = feat_in // 2
        self.norm_layer = norm_layer
        self.bin_size = bin_size
        self.dropout = nn.Dropout2d(0.1)
        self.conv_cam = nn.Conv2d(feat_in, num_classes, kernel_size=1)
        self.pool_cam = nn.AdaptiveAvgPool2d(bin_size)
        self.sigmoid = nn.Sigmoid()
        bin_num = bin_size[0] * bin_size[1]
        self.gcn = GCN(bin_num, feat_in)
        self.fuse = nn.Conv2d(bin_num, 1, kernel_size=1)
        self.proj_query = nn.Linear(feat_in, feat_inner)
        self.proj_key = nn.Linear(feat_in, feat_inner)
        self.proj_value = nn.Linear(feat_in, feat_inner)
        self.conv_out = nn.Sequential(
            nn.Conv2d(feat_inner, feat_in, kernel_size=1, bias=False),
            norm_layer(feat_in),
            nn.PReLU(feat_in),
        )
        self.scale = feat_inner ** -0.5
        self.relu = nn.PReLU(1)

    def forward(self, tensor):
        cam = self.conv_cam(tensor)
        class_score = self.sigmoid(self.pool_cam(cam))
        residual = tensor
        cam = patch_split(cam, self.bin_size)
        tensor = patch_split(tensor, self.bin_size)
        batch, _, region_h, region_w, classes = cam.shape
        channels = tensor.shape[-1]
        cam = cam.view(batch, -1, region_h * region_w, classes)
        tensor = tensor.view(batch, -1, region_h * region_w, channels)
        bin_confidence = class_score.view(batch, classes, -1).transpose(1, 2).unsqueeze(3)
        pixel_confidence = functional.softmax(cam, dim=2)
        local_features = torch.matmul(pixel_confidence.transpose(2, 3), tensor)
        local_features = self.gcn(local_features * bin_confidence)
        global_features = self.relu(self.fuse(local_features))
        global_features = global_features.repeat(1, tensor.shape[1], 1, 1)
        query = self.proj_query(tensor)
        key = self.proj_key(local_features)
        value = self.proj_value(global_features)
        affinity = functional.softmax(torch.matmul(query, key.transpose(2, 3)), dim=-1)
        output = torch.matmul(affinity, value)
        output = output.view(batch, -1, region_h, region_w, value.shape[-1])
        output = patch_recover(output, self.bin_size)
        return residual + self.conv_out(output)


class ConvBatchnormRelu(nn.Module):
    def __init__(self, n_in, n_out, kernel_size=3, stride=1, groups=1,
                 dropout_rate=0.0):
        super().__init__()
        padding = int((kernel_size - 1) / 2)
        self.conv = nn.Conv2d(
            n_in, n_out, kernel_size, stride=stride, padding=padding,
            bias=False, groups=groups)
        self.bn = nn.BatchNorm2d(n_out)
        self.act = nn.PReLU(n_out)
        self.dropout = nn.Dropout2d(dropout_rate) if dropout_rate > 0 else None

    def forward(self, tensor):
        output = self.act(self.bn(self.conv(tensor)))
        return self.dropout(output) if self.dropout else output


class DilatedConv(nn.Module):
    def __init__(self, n_in, n_out, kernel_size, stride=1, dilation=1, groups=1):
        super().__init__()
        padding = int((kernel_size - 1) / 2) * dilation
        self.conv = nn.Conv2d(
            n_in, n_out, kernel_size, stride=stride, padding=padding,
            bias=False, dilation=dilation, groups=groups)

    def forward(self, tensor):
        return self.conv(tensor)


class BatchnormRelu(nn.Module):
    def __init__(self, n_out):
        super().__init__()
        self.nOut = n_out
        self.bn = nn.BatchNorm2d(n_out, eps=1e-3)
        self.act = nn.PReLU(n_out)

    def forward(self, tensor):
        return self.act(self.bn(tensor))


class DepthwiseSeparableConv(nn.Module):
    def __init__(self, n_in, n_out, kernel_size=3, stride=1, dilation=1):
        super().__init__()
        padding = int((kernel_size - 1) / 2) * dilation
        self.depthwise = nn.Conv2d(
            n_in, n_in, kernel_size, stride, padding, dilation,
            groups=n_in, bias=False)
        self.pointwise = nn.Conv2d(n_in, n_out, 1, 1, 0, 1, 1, bias=False)

    def forward(self, tensor):
        return self.pointwise(self.depthwise(tensor))


class StrideESP(nn.Module):
    def __init__(self, n_in, n_out):
        super().__init__()
        channels = int(n_out / 5)
        first_channels = n_out - 4 * channels
        self.c1 = DilatedConv(n_in, channels, 3, 2)
        self.d1 = DilatedConv(channels, first_channels, 3, 1, 1)
        self.d2 = DilatedConv(channels, channels, 3, 1, 2)
        self.d4 = DilatedConv(channels, channels, 3, 1, 4)
        self.d8 = DilatedConv(channels, channels, 3, 1, 8)
        self.d16 = DilatedConv(channels, channels, 3, 1, 16)
        self.bn = nn.BatchNorm2d(n_out, eps=1e-3)
        self.act = nn.PReLU(n_out)

    def forward(self, tensor):
        reduced = self.c1(tensor)
        dilated_1 = self.d1(reduced)
        added_1 = self.d2(reduced)
        added_2 = added_1 + self.d4(reduced)
        added_3 = added_2 + self.d8(reduced)
        added_4 = added_3 + self.d16(reduced)
        merged = torch.cat([dilated_1, added_1, added_2, added_3, added_4], 1)
        return self.act(self.bn(merged))


class DepthwiseESP(nn.Module):
    def __init__(self, n_in, n_out, add=True):
        super().__init__()
        channels = max(int(n_out / 5), 1)
        first_channels = max(n_out - 4 * channels, 1)
        self.c1 = DepthwiseSeparableConv(n_in, channels, 1, 1)
        self.d1 = DepthwiseSeparableConv(channels, first_channels, 3, 1, 1)
        self.d2 = DepthwiseSeparableConv(channels, channels, 3, 1, 2)
        self.d4 = DepthwiseSeparableConv(channels, channels, 3, 1, 4)
        self.d8 = DepthwiseSeparableConv(channels, channels, 3, 1, 8)
        self.d16 = DepthwiseSeparableConv(channels, channels, 3, 1, 16)
        self.bn = BatchnormRelu(n_out)
        self.add = add

    def forward(self, tensor):
        reduced = self.c1(tensor)
        dilated_1 = self.d1(reduced)
        added_1 = self.d2(reduced)
        added_2 = added_1 + self.d4(reduced)
        added_3 = added_2 + self.d8(reduced)
        added_4 = added_3 + self.d16(reduced)
        merged = torch.cat([dilated_1, added_1, added_2, added_3, added_4], 1)
        return self.bn(tensor + merged if self.add else merged)


class AvgDownsampler(nn.Module):
    def __init__(self, sampling_times):
        super().__init__()
        self.pool = nn.ModuleList([
            nn.AvgPool2d(3, stride=2, padding=1) for _ in range(sampling_times)
        ])

    def forward(self, tensor):
        for pool in self.pool:
            tensor = pool(tensor)
        return tensor


class Encoder(nn.Module):
    def __init__(self, config):
        super().__init__()
        model_config = CHANNEL_CONFIGS[config]
        channels = model_config["channels"]
        self.level1 = ConvBatchnormRelu(3, channels[0], stride=2)
        self.sample1 = AvgDownsampler(1)
        self.sample2 = AvgDownsampler(2)
        self.b1 = ConvBatchnormRelu(channels[0] + 3, channels[1])
        self.level2_0 = StrideESP(channels[1], channels[2])
        self.level2 = nn.ModuleList([
            DepthwiseESP(channels[2], channels[2]) for _ in range(model_config["p"])
        ])
        self.b2 = ConvBatchnormRelu(channels[3] + 3, channels[3] + 3)
        self.level3_0 = StrideESP(channels[3] + 3, channels[3])
        self.level3 = nn.ModuleList([
            DepthwiseESP(channels[3], channels[3]) for _ in range(model_config["q"])
        ])
        self.b3 = ConvBatchnormRelu(channels[4], channels[2])

    def forward(self, tensor):
        level_1 = self.level1(tensor)
        input_1 = self.sample1(tensor)
        input_2 = self.sample2(tensor)
        level_1_cat = self.b1(torch.cat([level_1, input_1], 1))
        level_2_initial = self.level2_0(level_1_cat)
        level_2 = level_2_initial
        for layer in self.level2:
            level_2 = layer(level_2)
        level_2_cat = self.b2(torch.cat([level_2, level_2_initial, input_2], 1))
        level_3_initial = self.level3_0(level_2_cat)
        level_3 = level_3_initial
        for layer in self.level3:
            level_3 = layer(level_3)
        encoded = self.b3(torch.cat([level_3_initial, level_3], 1))
        return encoded, input_1, input_2


class UpSimpleBlock(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.deconv = nn.ConvTranspose2d(
            in_channels, out_channels, 2, stride=2, padding=0,
            output_padding=0, bias=False)
        self.bn = nn.BatchNorm2d(out_channels, eps=1e-3)
        self.act = nn.PReLU(out_channels)

    def forward(self, tensor):
        return self.act(self.bn(self.deconv(tensor)))


class UpConvBlock(nn.Module):
    def __init__(self, in_channels, out_channels, sub_dim=3, last=False,
                 kernel_size=3):
        super().__init__()
        self.last = last
        self.up_conv = UpSimpleBlock(in_channels, out_channels)
        if not last:
            self.conv1 = ConvBatchnormRelu(
                out_channels + sub_dim, out_channels, kernel_size)
        self.conv2 = ConvBatchnormRelu(out_channels, out_channels, kernel_size)

    def forward(self, tensor, original_image=None):
        tensor = self.up_conv(tensor)
        if not self.last:
            tensor = self.conv1(torch.cat([tensor, original_image], dim=1))
        return self.conv2(tensor)


class DrivableTwinLiteNetPlus(nn.Module):
    """TwinLiteNetPlus with an optional lane-line decoder."""

    def __init__(self, config="medium", enable_lane=False):
        super().__init__()
        if config not in CHANNEL_CONFIGS:
            raise ValueError("unknown model config: %s" % config)
        channels = CHANNEL_CONFIGS[config]["channels"]
        self.encoder = Encoder(config)
        self.caam = CAAM(
            feat_in=channels[2], num_classes=channels[2], bin_size=(2, 4),
            norm_layer=nn.BatchNorm2d)
        self.conv_caam = ConvBatchnormRelu(channels[2], channels[1])
        self.up_1_da = UpConvBlock(channels[1], channels[0])
        self.up_2_da = UpConvBlock(channels[0], 8)
        self.out_da = UpConvBlock(8, 2, last=True)
        self.enable_lane = bool(enable_lane)
        if self.enable_lane:
            self.up_1_ll = UpConvBlock(channels[1], channels[0])
            self.up_2_ll = UpConvBlock(channels[0], 8)
            self.out_ll = UpConvBlock(8, 2, last=True)

    def forward(self, tensor):
        encoded, input_1, input_2 = self.encoder(tensor)
        attended = self.conv_caam(self.caam(encoded))
        drivable = self.up_1_da(attended, input_2)
        drivable = self.up_2_da(drivable, input_1)
        drivable = self.out_da(drivable)
        if not self.enable_lane:
            return drivable
        lane = self.up_1_ll(attended, input_2)
        lane = self.up_2_ll(lane, input_1)
        lane = self.out_ll(lane)
        return drivable, lane
