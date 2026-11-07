"""Upload the dataset folder to a Hugging Face dataset repository (in a Qt worker thread).

Needs `pip install --user "huggingface_hub<1"` and a login (`huggingface-cli login`)
or a token typed into the dialog. Imported lazily so the GUI runs without it.
"""
from PyQt5.QtCore import QThread, pyqtSignal

INSTALL_HINT = 'pip install --user "huggingface_hub<1"'


def hub_available():
    try:
        import huggingface_hub  # noqa: F401
        return True
    except ImportError:
        return False


class UploadWorker(QThread):
    finished_ok = pyqtSignal(str)   # repository URL
    failed = pyqtSignal(str)        # error message

    def __init__(self, folder, repo_id, private, token=None):
        super().__init__()
        self.folder, self.repo_id, self.private = str(folder), repo_id, private
        self.token = token or None

    def run(self):
        try:
            from huggingface_hub import HfApi
            api = HfApi(token=self.token)
            api.create_repo(self.repo_id, repo_type='dataset', private=self.private, exist_ok=True)
            api.upload_folder(folder_path=self.folder, repo_id=self.repo_id, repo_type='dataset',
                              commit_message='Upload tactile episodes from hand_gui')
            self.finished_ok.emit(f'https://huggingface.co/datasets/{self.repo_id}')
        except Exception as exc:  # network, auth, permission ... shown to the user as-is
            self.failed.emit(f'{type(exc).__name__}: {exc}')
