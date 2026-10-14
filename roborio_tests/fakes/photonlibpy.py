"""Stand-in for photonlibpy."""


class PhotonCamera:
    def __init__(self, cameraName):
        self.name = cameraName

    def getAllUnreadResults(self):
        return []
