
class Recorder :
    def __init__(self, name, folder, dtopic, renable = True) -> None:
        self._name = name
        self._folder = folder is not None and isinstance(folder, str) if folder else name
        self.data_topic = dtopic
        self.enable_recorder = renable

    @property
    def name(self) -> str:
        return self._name

    @property
    def folder(self) -> str:
        return self._folder
    
    def __str__(self) -> str:
        return self.name

    def __repr__(self) -> str:
        return self.__str__()

    def begin_recording ():
        pass

    def record (data):
        pass

    def end_recording ():
        pass

