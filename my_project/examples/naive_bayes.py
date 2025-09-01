# -*- coding: utf-8 -*-

import numpy as np
from sklearn import naive_bayes as nb

X = np.random.randint(5, size=(6, 100))
y = np.array([1, 2, 3, 4, 5, 6])

classifier = np.MultinomialNB()

clf.fit(X,y)