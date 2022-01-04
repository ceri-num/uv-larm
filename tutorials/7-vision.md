# Introduction en traitement et analyse des images pour des applications de robotique

## Mise en place de l'environnement
 range(0, len(neigh_ind[0])):
        one_img=img_data[neigh_ind[0][j],:]
        r =
Python 3, OpenCV, Linux
Python librarie: OpenCV, Numpy, Matplot, Sklearn, Scipy

Tous ces outils ne sont pas nécessairement installés sur vos PC. Par conséquent, les actions suivantes sont à réaliser.
Vous devez être sudo sur vos machines. Si ce n'est pas le cas, il vous faudra créer en environnement virtuel dans lequel vous aurez toute liberté d'installer les librairies Python3 que vous allez utiliser.
 range(0, len(neigh_ind[0])):
        one_img=img_data[neigh_ind[0][j],:]
        r =
Sous python, l'outil pip permet d'installer les librairies. Cet outil devrait range(0, len(neigh_ind[0])):
        one_img=img_data[neigh_ind[0][j],:]
        r =  avoir été installé préalablement mais rien n'est moins sûr. Si ce n'est pas le cas, il faudra le faire ainsi (avec les droits sudo ... certains peuvent avoir ces droits et d'autres non) :
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
## Gestion de la souris

Voici quelques lignes de codes pour extraire une région d'intérêt à la souris. Grâce à ces quelques lignes il vous sera possible de calculer la valeur moyenne et la variance de chaque composante de l'ima range(0, len(neigh_ind[0])):
        one_img=img_data[neigh_ind[0][j],:]
        r = ge, utile pour procéder ensuite à une étape de segmentation.

```
import cv2
import numpy as np

if __name__ == '__main__' :

    cap=cv2.VideoCapture(0)

    # capture an image
    ret, frame=cap.read()

    # Select ROI
    r = cv2.selectROI(frame)

    # Crop image
    imCrop = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

    # Display cropped image
    cv2.imshow("Image", imCrop)
    cv2.waitKey(0)
```

Voici quelques lignes de codes pour gérer des actions sur la souris. Elles gères les événements souris tels que le mouvement de la souris (), le double click milieu (EVENT_MBUTTONDBLCLK), le click droit (EVENT_RBUTTONDOWN) et le click gauche (EVENT_LBUTTONDOWN).

```
import cv2
import numpy as np

def souris(event, x, y, flags, param):
    global lo, hi, color, hsv_px

    if event == cv2.EVENT_MOUSEMOVE:
        # Conversion des trois couleurs RGB sous la souris en HSV
        px = frame[y,x]
        px_array = np.uint8([[px]])
        hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)

    if event==cv2.EVENT_MBUTTONDBLCLK:
        color=image[y, x][0]

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>5:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=1

    lo[0]=color-5
    hi[0]=color+5

color=100

lo=np.array([color-5, 100, 50])
hi=np.array([color+5, 255,255])

color_info=(0, 0, 255)

cap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)
hsv_px = [0,0,0]

while True:
    ret, frame=cap.read()
    image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(image, lo, hi)
    mask=cv2.erode(mask, None, iterations=1)
    mask=cv2.dilate(mask, None, iterations=1)
    image2=cv2.bitwise_and(frame, frame, mask= mask)
    cv2.putText(frame, "Couleur: {:d}".format(color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

    # Affichage des composantes HSV sous la souris sur l'image
    pixel_hsv = " ".join(str(values) for values in hsv_px)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260),
               font, 1, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.imshow('Camera', frame)
    cv2.imshow('image2', image2)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
```

## Segmentation d'images couleur par seuillage des composantes

Généralement il est très intéressante de changer d'espace colorimétrique afin de mieux cibler l'espace dans lequel l'objet d'intérêt est discriminable : ```image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)```
Après avoir produite le mask avec ```mask=cv2.inRange(image, lo, hi)``` il est parfois pertinant de débruiter l'image résultats en lissant ou par quelques opérations motrphologiques. Cela permet de fermer et remplir les formes :
```
image=cv2.blur(image, (7, 7))
mask=cv2.erode(mask, None, iterations=4)
mask=cv2.dilate(mask, None, iterations=4)
```
Détecter les éléments connexes dans le mask puis extraire les informations relatives à chaque forme extraite pounr leur visualisation :
```
elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
if len(elements) > 0:
    c=max(elements, key=cv2.contourArea)
    ((x, y), rayon)=cv2.minEnclosingCircle(c)
    if rayon>30:
        cv2.circle(image2, (int(x), int(y)), int(rayon), color_infos, 2)
        cv2.circle(frame, (int(x), int(y)), 5, color_infos, 10)
        cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_infos, 2)
        cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_infos, 1, cv2.LINE_AA)
 ```

 Reste ensuite à visualiser les images.


## Segmentation des images par la méthodes des k-moyennes (kmeans)

Kmeans est un algorithme de clustering, dont l'objectif est de partitionner n points de données en k grappes. Chacun des n points de données sera assigné à un cluster avec la moyenne la plus proche. La moyenne de chaque groupe s'appelle «centroïde» ou «centre». Globalement, l'application de k-means donne k grappes distinctes des n points de données d'origine. Les points de données à l'intérieur d'un cluster particulier sont considérés comme «plus similaires» les uns aux autres que les points de données appartenant à d'autres groupes. Cet algorithme peut être appliquer sur des points d’origine géométrique, colorimétriques et autres.

Nous allons appliquer cette méthode afin d'assurer une segmentation couleur d'une image i.e. cela revient à trouver les couleur domainantes dans l'image.
```
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import cv2
import numpy as np

#Ensuite charger une image et la convertir de BGR à RGB si nécessaire et l’afficher :
image = cv2.imread('lena.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
plt.figure()
plt.axis("off")
plt.imshow(image)
```
Afin de traiter l’image en tant que point de données, il faut la convertir d’une forme matricielle à une forme vectorielle (liste de couleur rgb) avant d'appliquer la fonction de clustering :
```
n_clusters=5
image = image.reshape((image.shape[0] * image.shape[1], 3))
clt = KMeans(n_clusters = n_clusters )
clt.fit(image)
```
Pour afficher les couleurs les plus dominantes dans l'image, il faut définir deux fonctions : centroid_histogram() pour récupérer le nombre de clusters différents et créer un histogramme basé sur le nombre de pixels affectés à chaque cluster ; et plot_colors() pour initialiser le graphique à barres représentant la fréquence relative de chacune des couleurs
```
def centroid_histogram(clt):
    numLabels = np.arange(0, len(np.unique(clt.labels_)) + 1)
    (hist, _) = np.histogram(clt.labels_, bins=numLabels)

    # normalize the histogram, such that it sums to one
    hist = hist.astype("float")
    hist /= hist.sum()

    return hist

def plot_colors(hist, centroids):
    bar = np.zeros((50, 300, 3), dtype="uint8")
    startX = 0

    # loop over the percentage of each cluster and the color of
    # each cluster
    for (percent, color) in zip(hist, centroids):
        # plot the relative percentage of each cluster
        endX = startX + (percent * 300)
        cv2.rectangle(bar, (int(startX), 0), (int(endX), 50),
                      color.astype("uint8").tolist(), -1)
        startX = endX

    return bar
```
Il suffit maintenant de construire un histogramme de clusters puis créer une figure représentant le nombre de pixels étiquetés pour chaque couleur.
```
hist = centroid_histogram(clt)
bar = plot_colors(hist, clt.cluster_centers_)
plt.figure()
plt.axis("off")
plt.imshow(bar)
plt.show()
```

## Classification d'images par la mathode des K plus proches voisins (k-NN ou KNN)

Cet exercice permettra d'apprendre un modèle à partir des images de la bases CIFAR-10 téléchargeable ici:
http://www.cs.toronto.edu/~kriz/cifar.html
Décompresser les fichier dans un dossier que vous utiliserez dans le script suivant.
Ici, le dossier est ./data

```
import numpy as np
import cv2

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
Attention, les composantes RGB des images sont arrangées sous la forme d'une vecteur à 1 dimension.
Pour afficher chaque image, il faut donc remettre sous la forme d'une image 2D RGB.
Pour cela, nous opérons de la manière suivante en considérant que les images sont de résolution 32x32

```
one_img=sample_img_data[0,:]
r = one_img[:1024].reshape(32, 32)
g = one_img[1024:2048].reshape(32, 32)
b = one_img[2048:].reshape(32, 32)
rgb = np.dstack([r, g, b])
cv2.imshow('Image CIFAR',rgb)
cv2.waitKey(0)
cv2.destroyAllWindows()
```
Désormais, nous allons appliquer l'algorithmes des k-NN sur toutes les images de la base de training img_data et leurs labels img_label_orig
```
from sklearn.neighbors import KNeighborsClassifier

#def pred_label_fn(i, original):
#    return original + '::' + meta[YPred[i]].decode('utf-8')

nbrs = KNeighborsClassifier(n_neighbors=3, algorithm='brute').fit(img_data, img_label_orig)

# test sur les 10 premières images
data_point_no = 10
sample_test_data = test_data[:data_point_no, :]

YPred = nbrs.predict(sample_test_data)

for i in range(0, len(YPred)):
    #show_im(sample_test_data, test_label, meta, i, label_fn=pred_label_fn)
    r = sample_test_data[i][:1024].reshape(32, 32)
    g = sample_test_data[i][1024:2048].reshape(32, 32)
    b = sample_test_data[i][2048:].reshape(32, 32)
    print(YPred[i])
    cv2.imshow('image test',np.dstack([r, g, b]))

    neigh_dist,neigh_ind = nbrs.kneighbors([sample_test_data[i]])
    print(neigh_ind)
    for j in range(0, len(neigh_ind[0])):
        one_img=img_data[neigh_ind[0][j],:]
        r = one_img[:1024].reshape(32, 32)
        g = one_img[1024:2048].reshape(32, 32)
        b = one_img[2048:].reshape(32, 32)
        rgb = np.dstack([r, g, b])
        cv2.imshow('K plus proche image',np.dstack([r, g, b]))
        cv2.waitKey(0)
```

## Détection d'objets par ondelettes de Haar

La détection d'objets à l'aide de classificateurs en cascade basés sur la décomposition en ondelettes de Haar est une méthode efficace de détection d'objets proposée par Paul Viola et Michael Jones dans leur article, "Rapid Object Detection using a Boosted Cascade of Simple Features" en 2001. Il s'agit d'une approche basée sur l'apprentissage automatique où un la fonction en cascade est formée à partir d'un grand nombre d'images positives et négatives.
Cette méthode a été initialement mise en au point pour détecter des visages et a été étendu à d'autres objets tels quels les voitures.

En python, vous pouvez faire appel à cette méthode via ``` object_cascade=cv2.CascadeClassifier() ```. Cette classe est instanciée en lui passant un paramètre qui représente le "modèle" adapté à l'objet à détecter.
Vous pouvez télécharger les modèles relatifs à des humains ici : https://github.com/opencv/opencv/tree/master/data/haarcascades
Pour tester le détecteur sur des véhicules, le modèle proposé par Andrews Sobral est téléchrgeable ici : https://github.com/andrewssobral/vehicle_detection_haarcascades/blob/master/cars.xml

Pour appliquer le détecteur à une image il suffit d'appeler la méthode ```object=object_cascade.detectMultiScale(gray, scaleFactor=1.10, minNeighbors=3)``` en passant en paramètre le nom de la variable image (gray) qu'il faut préalablement transformée en niveau de gris. Il fauat également renseigner le facteur d'échelle (scaleFactor) utilisé pour réduire l'image à chaque étage et le nombre de voisins (minNeighbors) que chaque objet détecté doit avoir pour le valider comme "effectivement" l'objet recherché.

Cette méthode fournit une liste de boites englobantes (x, y, w et h) que vous afficherez sur chaque image couleur traitée afin de visualiser les résultats de la détection.

```
for x, y, w, h in object:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
```

Ecrire un script permettant de mettre en musique cette classe et cette méthode sur la vidéo cars.mp4 fournies.
Vous validerez votre script en utilisant les modèles relatifs au corps humains et en utilisant le flux d'une caméra.

### Model training

Cette méthode pourrait être très intéressante pour détecter des objets lors du "challenge". Pour cela, je vous invite à lire et utiliser ce qui est proposé sur les 2 liens suivants. Ces liens décrivent comment il est possible d'apprendre un modèle spécifique à un objet donné.

http://coding-robin.de/2013/07/22/train-your-own-opencv-haar-classifier.html

https://github.com/mrnugget/opencv-haar-classifier-training

### Model training for an other feature

Vous trouverez dans le lien suivant, l'apprentissage d'un modèle sur la base d'un autre type de caractéreristique : les Local Binary Pattern (LBP).

https://medium.com/@rithikachowta/object-detection-lbp-cascade-classifier-generation-a1d1a1c2d0b

## Extraction de régions dans une image binarisée

Voici quelques lignes en python pour extraire des région de pixels connexes dans une image binarisée ``` label()```.
De ces régions sont extraites quelques propriétés ``` regionprops()```

```
import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops
import math

image = cv2.imread('./vector-handwritten-numbers-on-white-background-brusk-stroke.jpg')

# passage en niveau de gris
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

###### extration des régions avec la lib skimage

# Binarisation de l'image
ret, thresh = cv2.threshold(gray, 127, 255, 1)
cv2.imshow("image seuillée",thresh)
cv2.waitKey(0)

# extraction des régions et des propriétés des régions
label_img = label(thresh)
regions = regionprops(label_img)
print(regions)
cv2.waitKey(0)

# affichage des régions et des boites englobantes
fig, ax = plt.subplots()
ax.imshow(thresh, cmap=plt.cm.gray)

for props in regions:
    y0, x0 = props.centroid
    orientation = props.orientation
    x1 = x0 + math.cos(orientation) * 0.5 * props.minor_axis_length
    y1 = y0 - math.sin(orientation) * 0.5 * props.minor_axis_length
    x2 = x0 - math.sin(orientation) * 0.5 * props.major_axis_length
    y2 = y0 - math.cos(orientation) * 0.5 * props.major_axis_length

    ax.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
    ax.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
    ax.plot(x0, y0, '.g', markersize=15)

    minr, minc, maxr, maxc = props.bbox
    bx = (minc, maxc, maxc, minc, minc)
    by = (minr, minr, maxr, maxr, minr)
    ax.plot(bx, by, '-b', linewidth=2.5)

ax.axis((0, 600, 600, 0))
plt.show()

cv2.waitKey(0)
```
