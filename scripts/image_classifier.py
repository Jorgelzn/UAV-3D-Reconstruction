import tensorflow as tf
import numpy as np
import cv2
import json
import os
from tqdm import tqdm

def train():
    from tensorflow.keras.applications.vgg16 import VGG16
    from tensorflow.keras.applications.vgg16 import preprocess_input

    dataset_path = os.path.join(os.path.dirname(__file__),"data","ARID_imagenet")

    train_datagen= tf.keras.preprocessing.image.ImageDataGenerator(
            rescale=1./255,
            validation_split=0.2,
            )

    train_generator = train_datagen.flow_from_directory(
        directory=dataset_path,
        target_size=(64,64),
        color_mode="rgb",
        class_mode="categorical",
        batch_size=16,
        subset='training'
    )

    validation_generator = train_datagen.flow_from_directory(
        directory=dataset_path,
        target_size=(64,64),
        batch_size=16,
        color_mode="rgb",
        class_mode='categorical',
        subset='validation') 

    """
        rotation_range=40,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
        fill_mode='nearest'
    """

    label_map = train_generator.class_indices

    with open(os.path.join(os.path.dirname(__file__),"data","images_class_info.json"), 'w') as file:
        file.write(json.dumps(label_map))

    vgg_model = VGG16(weights="imagenet", include_top=False, input_shape=(64, 64, 3))
    vgg_model.trainable = False 

    model = tf.keras.Sequential([
        vgg_model,          
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(256, activation="relu"),
        tf.keras.layers.Dense(5,activation="softmax")
    ])

    model.compile(optimizer="adam",loss="categorical_crossentropy",metrics="accuracy")

    print(model.summary())

    STEP_SIZE_TRAIN=train_generator.n//train_generator.batch_size

    history=model.fit_generator(generator=train_generator,
                        validation_data=validation_generator,
                        steps_per_epoch=STEP_SIZE_TRAIN,
                        epochs=3)

    model.evaluate_generator(generator=validation_generator,steps=10)

    model.save(os.path.join(os.path.dirname(__file__),"data","image_classification_model.h5"))


def classify(object_dir):

    images_path = os.path.join(object_dir,"images")

    test_model = tf.keras.models.load_model(os.path.join(os.path.dirname(__file__),"data","image_classification_model.h5"))

    with open(os.path.join(os.path.dirname(__file__),"data","images_class_info.json")) as json_file:
        class_info = json.load(json_file)
        object_count = [0]*len(class_info)
        prob_count = [0]*len(class_info)

    for image in tqdm(os.listdir(images_path)):
        img = cv2.imread(os.path.join(images_path,image), cv2.IMREAD_COLOR)
        img = cv2.resize(img, (64,64))
        img=img.reshape(1,64,64,3)

        prediction = test_model.predict(img)

        object_count[prediction.argmax()]+=1
        prob_count[prediction.argmax()]+=prediction.max()

    object_type = list(class_info)[np.argmax(object_count)]
    probability = max(prob_count)/len(os.listdir(images_path))

    with open(os.path.join(object_dir,"object_data.txt"), "a") as file:
        file.write(object_type+" with a mean probability of "+str(probability))

