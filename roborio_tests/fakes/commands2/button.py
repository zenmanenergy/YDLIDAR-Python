"""Stand-in for commands2.button: records which command is bound to which button."""


class Trigger:
    def __init__(self, name):
        self.name = name
        self.bindings = {}

    def onTrue(self, command):
        self.bindings["onTrue"] = command
        return self

    def whileTrue(self, command):
        self.bindings["whileTrue"] = command
        return self


class CommandXboxController:
    def __init__(self, port):
        self.port = port
        self.triggers = {}

    def _trigger(self, name):
        return self.triggers.setdefault(name, Trigger(name))

    def a(self, loop=None):
        return self._trigger("a")

    def b(self, loop=None):
        return self._trigger("b")
