# Introduction en traitement et analyse des images pour des applications de robotique

Python 3, OpenCV, Linux
Python librarie: OpenCV, Numpy, Matplot, Sklearn, Scipy

Tous ce outils ne sont pas nécessairement installés sur vos PC. Par conséquent, les actions suivantes sont à réaliser.
Puisque vous n'êtes pas sudo sur vos PC, il vous faudra créer en environnement virtuel dans lequel vous aurez toute liberté d'installer les librairies Python3 que vous allez utiliser.

Sou python, l'outil pip permet d'installer les librairies. Cet outil devrait avoir été installé préalablement mais rien n'est moins sûr. Si ce n'est pas le cas, il faudra le faire ainsi (avec les droits sudo ... certains peuvent avoir ces droits et d'autres non) :
```
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
```

Lorsque pip est installé alors il vous faudra installer les modules suivants :

```
sudo pip3 install --proxy=http://10.100.1.4:8080 numpy, tensorflow, opencv-python, opencv-contrib-python, sklearn, scipy
```

Petit rappel - l'utilisation de ces modules dans vos scripts est réalisé par exempe ainsi :
```
import cv2
import numpy as np
import os
from sklearn.svm import LinearSVC
from sklearn.externals import joblib
from scipy.cluster.vq import *
from sklearn.preprocessing import StandardScaler
from sklearn import preprocessing
```
