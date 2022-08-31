#!/bin/bash

media-ctl -r
media-ctl -d /dev/media0 -V "'rcar_csi2 feaa0000.csi2':1 [fmt:UYVY8_2X8/1280x800 field:none]"
media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':1 -> 'VIN0 output':0 [1]"
media-ctl -d /dev/media0 -V "'rcar_csi2 feaa0000.csi2':2 [fmt:UYVY8_2X8/1280x800 field:none]"
media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':2 -> 'VIN1 output':0 [1]"
media-ctl -d /dev/media0 -V "'rcar_csi2 feaa0000.csi2':3 [fmt:UYVY8_2X8/1280x800 field:none]"
media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':3 -> 'VIN2 output':0 [1]"
media-ctl -d /dev/media0 -V "'rcar_csi2 feaa0000.csi2':4 [fmt:UYVY8_2X8/1280x800 field:none]"
media-ctl -d /dev/media0 -l "'rcar_csi2 feaa0000.csi2':4 -> 'VIN3 output':0 [1]"
