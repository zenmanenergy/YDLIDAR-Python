"""Stand-in for commands2 with the same method names."""


class Subsystem:
    def __init__(self):
        pass

    def periodic(self):
        pass


class Command:
    def __init__(self):
        self.requirements = set()

    def addRequirements(self, *requirements):
        self.requirements.update(requirements)

    def initialize(self):
        pass

    def execute(self):
        pass

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass


class SequentialCommandGroup(Command):
    def __init__(self, *commands):
        super().__init__()
        self.commands = list(commands)


def _and_then(self, *next_commands):
    return SequentialCommandGroup(self, *next_commands)


Command.andThen = _and_then


class TimedCommandRobot:
    def __init__(self):
        pass
