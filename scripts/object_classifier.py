import os
import glob
import trimesh
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from matplotlib import pyplot as plt
import open3d as o3d
import json

DATA_DIR = os.path.join(os.path.expanduser('~'), 'Documents', 'AirSim',"ModelNet40")
NUM_POINTS = 6000
NUM_CLASSES = 3
BATCH_SIZE = 5

class OrthogonalRegularizer(keras.regularizers.Regularizer):
    def __init__(self, num_features, l2reg=0.001):
        self.num_features = num_features
        self.l2reg = l2reg
        self.eye = tf.eye(num_features)

    def __call__(self, x):
        x = tf.reshape(x, (-1, self.num_features, self.num_features))
        xxt = tf.tensordot(x, x, axes=(2, 2))
        xxt = tf.reshape(xxt, (-1, self.num_features, self.num_features))
        return tf.reduce_sum(self.l2reg * tf.square(xxt - self.eye))

def parse_dataset(num_points=2048):

    train_points = []
    train_labels = []
    test_points = []
    test_labels = []
    class_map = {}

    for i, folder in enumerate(os.listdir(DATA_DIR)):
        print("processing class: {}".format(folder))
        # store folder name with ID so we can retrieve later
        class_map[i] = folder
        # gather all files
        train_files = glob.glob(os.path.join(DATA_DIR,folder, "train/*"))
        test_files = glob.glob(os.path.join(DATA_DIR,folder, "test/*"))

        for f in train_files:
            train_points.append(trimesh.load(f).sample(num_points))
            train_labels.append(i)

        for f in test_files:
            test_points.append(trimesh.load(f).sample(num_points))
            test_labels.append(i)

    return (
        np.array(train_points),
        np.array(test_points),
        np.array(train_labels),
        np.array(test_labels),
        class_map,
    )

def augment(points, label):
    # jitter points
    points += tf.random.uniform(points.shape, -0.005, 0.005, dtype=tf.float64)
    # shuffle points
    points = tf.random.shuffle(points)
    return points, label

def conv_bn(x, filters):
    x = layers.Conv1D(filters, kernel_size=1, padding="valid")(x)
    x = layers.BatchNormalization(momentum=0.0)(x)
    return layers.Activation("relu")(x)


def dense_bn(x, filters):
    x = layers.Dense(filters)(x)
    x = layers.BatchNormalization(momentum=0.0)(x)
    return layers.Activation("relu")(x)

def tnet(inputs, num_features):

    # Initalise bias as the indentity matrix
    bias = keras.initializers.Constant(np.eye(num_features).flatten())
    reg = OrthogonalRegularizer(num_features)

    x = conv_bn(inputs, 32)
    x = conv_bn(x, 64)
    x = conv_bn(x, 512)
    x = layers.GlobalMaxPooling1D()(x)
    x = dense_bn(x, 256)
    x = dense_bn(x, 128)
    x = layers.Dense(
        num_features * num_features,
        kernel_initializer="zeros",
        bias_initializer=bias,
        activity_regularizer=reg,
    )(x)
    feat_T = layers.Reshape((num_features, num_features))(x)
    # Apply affine transformation to input features
    return layers.Dot(axes=(2, 1))([inputs, feat_T])


def train():

    train_points, test_points, train_labels, test_labels, CLASS_MAP = parse_dataset(NUM_POINTS)

    with open(os.path.join(os.path.dirname(__file__),"object_classifier_data","object_class_info.json"), 'w') as file:
        CLASS_MAP = {v: k for k, v in CLASS_MAP.items()}
        file.write(json.dumps(CLASS_MAP))

    train_dataset = tf.data.Dataset.from_tensor_slices((train_points, train_labels))
    test_dataset = tf.data.Dataset.from_tensor_slices((test_points, test_labels))

    train_dataset = train_dataset.shuffle(len(train_points)).map(augment).batch(BATCH_SIZE)
    test_dataset = test_dataset.shuffle(len(test_points)).batch(BATCH_SIZE)

    inputs = keras.Input(shape=(NUM_POINTS, 3))

    x = tnet(inputs, 3)
    x = conv_bn(x, 32)
    x = conv_bn(x, 32)
    x = tnet(x, 32)
    x = conv_bn(x, 32)
    x = conv_bn(x, 64)
    x = conv_bn(x, 512)
    x = layers.GlobalMaxPooling1D()(x)
    x = dense_bn(x, 256)
    x = layers.Dropout(0.3)(x)
    x = dense_bn(x, 128)
    x = layers.Dropout(0.3)(x)

    outputs = layers.Dense(NUM_CLASSES, activation="softmax")(x)

    model = keras.Model(inputs=inputs, outputs=outputs, name="pointnet")
    print(model.summary())

    model.compile(
        loss="sparse_categorical_crossentropy",
        optimizer=keras.optimizers.Adam(learning_rate=0.001),
        metrics=["sparse_categorical_accuracy"],
    )

    model.fit(train_dataset, epochs=1, validation_data=test_dataset)

    model.save_weights(os.path.join(os.path.dirname(__file__),"object_classifier_data","object_classification_model"))

def classify(object_path):

    inputs = keras.Input(shape=(NUM_POINTS, 3))

    x = tnet(inputs, 3)
    x = conv_bn(x, 32)
    x = conv_bn(x, 32)
    x = tnet(x, 32)
    x = conv_bn(x, 32)
    x = conv_bn(x, 64)
    x = conv_bn(x, 512)
    x = layers.GlobalMaxPooling1D()(x)
    x = dense_bn(x, 256)
    x = layers.Dropout(0.3)(x)
    x = dense_bn(x, 128)
    x = layers.Dropout(0.3)(x)

    outputs = layers.Dense(NUM_CLASSES, activation="softmax")(x)

    model = keras.Model(inputs=inputs, outputs=outputs, name="pointnet")


    model.load_weights(os.path.join(os.path.dirname(__file__),"object_classifier_data","object_classification_model"))

    with open(os.path.join(os.path.dirname(__file__),"object_classifier_data","object_class_info.json")) as json_file:
        class_info = json.load(json_file)

    points = trimesh.load(object_path).sample(NUM_POINTS)

    points = np.expand_dims(points, axis=0)

    preds = model.predict(points)
    index = np.argmax(preds)

    print("predited class:",list(class_info)[index],"with accuracy",preds[0][index])

    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(2, 4, 0 + 1, projection="3d")
    ax.scatter(points[0, :, 0], points[0, :, 1], points[0, :, 2])
    ax.set_title("pred: {:}, probability: {:}".format(list(class_info)[index], preds[0][index]))
    ax.set_axis_off()
    plt.show()
   
#train()
classify("C:/Users/jorge/Documents/AirSim/data/mission_0/object_0/object.obj")
