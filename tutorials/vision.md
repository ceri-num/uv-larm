# Introduction en traitement et analyse des images pour des applications de robotique

## Mise en place de l'environnement

Python 3, OpenCV, Linux
Python librarie: OpenCV, Numpy, Matplot, Sklearn, Scipy

Tous ces outils ne sont pas nécessairement installés sur vos PC. Par conséquent, les actions suivantes sont à réaliser.
Vous devez être sudo sur vos machines. Si ce n'est pas le cas, il vous faudra créer en environnement virtuel dans lequel vous aurez toute liberté d'installer les librairies Python3 que vous allez utiliser.

Sous python, l'outil pip permet d'installer les librairies. Cet outil devrait avoir été installé préalablement mais rien n'est moins sûr. Si ce n'est pas le cas, il faudra le faire ainsi (avec les droits sudo ... certains peuvent avoir ces droits et d'autres non) :
```
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
```

Lorsque pip est installé alors il vous faudra installer les modules suivants :

```
sudo pip3 install --proxy=http://10.100.1.4:8080 numpy tensorflow opencv-python==3.4.2.16 opencv-contrib-python==3.4.2.16 sklearn scipy matplotlib psutil
```
Petit rappel - l'utilisation de ces modules dans vos scripts est réalisé par exemple ainsi :
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
## Classification d'images par la mathode des K plus proches voisins (k-NN ou KNN)

Cet exercice permettra d'apprendre un modèle à partir des images de la bases CIFAR-10 téléchargeable ici:
http://www.cs.toronto.edu/~kriz/cifar.html
Décompresser les fichier dans un dossier que vous utiliserez dans le script suivant.
Ici, le dossier est ./data

```
basedir_data = "./data/"
rel_path = basedir_data + "cifar-10-batches-py/"

#Désérialiser les fichiers image afin de permettre l’accès aux données et aux labels:

def unpickle(file):
    import pickle
    with open(file, 'rb') as fo:
        dict = pickle.load(fo,encoding='bytes')
    return dict

X = unpickle(rel_path + 'data_batch_1')
img_data = X[b'data']
img_label_orig = img_label = X[b'labels']
img_label = np.array(img_label).reshape(-1, 1)
````
afin de vérifier que tout s'est bien passé utilisé :
```
print(img_data)
print('shape', img_data.shape)
```
Vous devriez trouver un tableau numpy de 10000x3072 d'uint8s (le 3072 vient du 3 x 1024). Chaque ligne du tableau stocke une image couleur 32x32 en RGB. L'image est stockée dans l'ordre des lignes principales, de sorte que les 32 premières entrées du tableau correspondent aux valeurs des canaux rouges de la première ligne de l'image.
Pour vérifier les labels :
```
print(img_label)
print('shape', img_label.shape)
```
Nous avons les étiquettes comme dane matrice 10000 x 1

Pour charger les données de test, utiliser la même procédure que précédement car la forme des données de test est identique à la forme des données d’apprentissage:
```
test_X = unpickle(rel_path + 'test_batch');
test_data = test_X[b'data']
test_label = test_X[b'labels']
test_label = np.array(test_label).reshape(-1, 1)
```
Vérifier que tout s'est bien déroulé comme précédement : deux tableaux numpy de respectivement 10000 x 3072 et 10000 x 1 élements. 
Pour extraire les a10 premières images de img_data et vérifier la taille du contenu de chaque élément, il suffit de faire ainsi :
```
sample_img_data = img_data[0:10, :]
print(sample_img_data)
print('shape', sample_img_data.shape)
print('shape', sample_img_data[1,:].shape)
````
Attention, les composantes RGB des images sont rangées en un vecteur d'1 dimension.
Pour afficher chaque image, il faut donc remettre sous la forme d'une image 2D RGB.
