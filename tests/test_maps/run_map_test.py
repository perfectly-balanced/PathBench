import sys
import os

# add 'PathBench/tests' to system path for module imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils import make_src_modules_importable  # noqa: E402

# add src folder to system path
make_src_modules_importable()

import test_dense_map as tst  # noqa: E402

t = tst.TestDenseMap()
t.test_convert_to_sparse_map()
