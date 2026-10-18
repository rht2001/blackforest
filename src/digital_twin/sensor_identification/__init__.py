"""Hardware-independent, conservative CAD compatibility matching."""
import os

# Fitting uses small matrices. Avoid a large BLAS thread pool in the local
# viewer process, while respecting an explicitly configured environment.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

from .classifier import IdentificationConfig, identify_sensor
from .geometry import CandidateModel, load_candidates

__all__ = ['IdentificationConfig', 'identify_sensor', 'CandidateModel', 'load_candidates']
