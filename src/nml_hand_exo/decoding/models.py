from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.preprocessing import StandardScaler


@dataclass
class ShrinkageLDAIntentModel:
    scaler: StandardScaler | None = None
    classifier: LinearDiscriminantAnalysis | None = None

    def fit(self, features: np.ndarray, labels: np.ndarray) -> "ShrinkageLDAIntentModel":
        X = np.asarray(features, dtype=np.float64)
        y = np.asarray(labels, dtype=object)
        classes = np.unique(y)
        if X.ndim != 2 or len(X) != len(y):
            raise ValueError("features and labels must contain matching rows")
        if len(classes) < 2:
            raise ValueError("At least two classes are required")
        self.scaler = StandardScaler().fit(X)
        self.classifier = LinearDiscriminantAnalysis(
            solver="lsqr",
            shrinkage="auto",
            priors=np.full(len(classes), 1.0 / len(classes)),
        ).fit(self.scaler.transform(X), y)
        return self

    @property
    def classes(self) -> tuple[str, ...]:
        if self.classifier is None:
            return ()
        return tuple(str(value) for value in self.classifier.classes_)

    def predict(self, features: np.ndarray) -> np.ndarray:
        if self.scaler is None or self.classifier is None:
            raise RuntimeError("Intent model has not been fit")
        return self.classifier.predict(self.scaler.transform(np.asarray(features, dtype=np.float64)))

    def predict_proba(self, features: np.ndarray) -> np.ndarray:
        if self.scaler is None or self.classifier is None:
            raise RuntimeError("Intent model has not been fit")
        return self.classifier.predict_proba(
            self.scaler.transform(np.asarray(features, dtype=np.float64))
        )
