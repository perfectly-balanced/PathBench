import torch
import warnings

from simulator.services.debug import DebugLevel
from simulator.services.service import Service


class Torch(Service):
    SEED: int = 42
    GPU: bool = True
    DEVICE_INDEX: int = 0

    def __init__(self, _services: 'Services'):
        super().__init__(_services)

        self.__device = None

        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", ".*CUDA initialization.*")
            self.__init_seed()
            self.__init_device()

    def __init_seed(self):
        if torch.cuda.is_available():
            torch.backends.cudnn.deterministic = True
        torch.manual_seed(self.SEED)

    def __init_device(self):
        if self.GPU:
            self.__device = torch.device(self.gpu if torch.cuda.is_available() else self.cpu)
        else:
            self.__device = torch.device(self.cpu)
        self._services.debug.write("Active torch device: " + str(self.__device), DebugLevel.LOW)

    @property
    def device(self) -> str:
        return "device"

    @device.getter
    def device(self):
        return self.__device

    @property
    def cpu(self) -> str:
        return "cpu"

    @cpu.getter
    def cpu(self) -> str:
        return "cpu"

    @property
    def gpu(self) -> str:
        return "gpu"

    @gpu.getter
    def gpu(self) -> str:
        return "cuda:" + str(self.DEVICE_INDEX)
