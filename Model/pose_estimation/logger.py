from tensorboardX import SummaryWriter
import os


class Logger:
    def __init__(self, *, log_dir, config):
        self.writer = SummaryWriter(log_dir, write_to_disk=True)
        self.config = config

    def log_training(
        self,
        *,
        training_metric_translation,
        training_metric_orientation,
        training_metric_scaleY,
        epoch,
    ):
        """
        Write the training translation and orientation error in the writer object to
        print it out on tensorboard for each epoch

        Args:
            training_metric_translation (float): error on the translation calculated on one training epoch
            training_metric_orientation (float): error on the orientation calculated on one training epoch
            epoch (int): number of the current epoch
        """
        # loss training
        self.writer.add_scalar(
            f"training/loss_translation",
            training_metric_translation,
            epoch,
        )
        self.writer.add_scalar(
            f"training/loss_scaleY",
            training_metric_scaleY,
            epoch,
        )

        if self.config.dataset.symmetric == False:
            self.writer.add_scalar(
                f"training/loss_orientation",
                training_metric_orientation,
                epoch,
            )

    def log_evaluation(
        self,
        *,
        evaluation_metric_translation,
        evaluation_metric_orientation,
        evaluation_metric_scaleY,
        epoch,
        test,
    ):
        """
        Write the evaluatin translation and orientation error in the writer object to
        print it out on tensorboard for each epoch. It is the validation translation and orientation
        error if we evaluate on the validation dataset and it is the test translation and orientation
        error if we evaluate it on the test set

        Args:
            evaluation_metric_translation (float): error on the translation calculated on one validation epoch
            evaluation_metric_orientation (float): error on the orientation calculated on one validation epoch
            epoch (int): number of the current epoch
            test (bool): specify if it is an evaluation in the training framework or in the test one.
        """

        if test:
            # loss test
            self.writer.add_scalar(
                f"test/loss_translation", evaluation_metric_translation
            )
            self.writer.add_scalar(
                f"test/loss_scaleY", evaluation_metric_scaleY
            )
            if self.config.dataset.symmetric == False:
                self.writer.add_scalar(
                    f"test/loss_orientation",
                    evaluation_metric_orientation,
                )
        else:
            # loss validation
            self.writer.add_scalar(
                f"val/loss_translation",
                evaluation_metric_translation,
                epoch,
            )
            self.writer.add_scalar(
                f"val/loss_scaleY", 
                evaluation_metric_scaleY,
                epoch,
            )

            if self.config.dataset.symmetric == False:
                self.writer.add_scalar(
                    f"val/loss_orientation",
                    evaluation_metric_orientation,
                    epoch,
                )

    def done(self):
        """
        Close the writer after the whole training + evaluation
        """
        self.writer.close()


# HELPER
def is_master():
    rank = int(os.getenv("RANK", 0))
    return rank == 0
