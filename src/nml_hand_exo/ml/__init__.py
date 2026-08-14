try:
    from ._models import EMGRegressor
    from ._model_manager import ModelManager
except ImportError as exc:  # Optional PyTorch compatibility helpers.
    raise ImportError(
        "nml_hand_exo.ml requires the optional ML dependencies. "
        "Install them with: pip install 'nml-hand-exo[ml]'"
    ) from exc

__all__ = ["EMGRegressor", "ModelManager"]
