#!/usr/bin/env python
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import tensorflow as tf

class ModelTrain:
    def __init__(self, filepath):
        # dim: 92 (input 90, output 2)
        columns = ['degree_%s'%i for i in range(45, 135)] + ['linear', 'angular']
        df = pd.read_csv(filepath)
        df.columns = columns

        # drop first 50 and last 50 data
        self.df = df.iloc[50:-50]
        
    def prepareData(self):
        self.df.replace([np.inf], 3.5, inplace=True)
        train, val = np.split(self.df.sample(frac=1), [int(0.8*len(self.df))]) 

        label_columns = ['linear', 'angular']
        self.train_y = train[label_columns].to_numpy()
        self.train_X = train.drop(columns=label_columns).to_numpy()
        self.val_y = val[label_columns].to_numpy()
        self.val_X = val.drop(columns=label_columns).to_numpy()

    def createModel(self):
        self.model = tf.keras.Sequential([
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dense(2)
        ])

        self.model.compile(optimizer=tf.keras.optimizers.Adam(lr=0.002), loss='mean_squared_error')

    def train(self):
        self.history = self.model.fit(self.train_X, self.train_y, epochs=100, \
                                      validation_data=(self.val_X, self.val_y))

    def saveModel(self, savepath):
        self.model.save(savepath)

if __name__=='__main__':
    filepath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir, 'data/record.csv'))
    
    m = ModelTrain(filepath)
    m.prepareData()
    m.createModel()

    # train
    m.train()

    # save model
    modelpath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir, 'model/my_model'))
    m.saveModel(modelpath)

