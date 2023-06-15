import torch
import torchvision

from pose_estimation.single_cube_dataset import SingleCubeDataset
from pose_estimation.evaluation_metrics.translation_average_mean_square_error import (
    translation_average_mean_square_error,
)


def evaluate_model(estimator):
    """
    Do the evaluation process for the estimator

    Args:
        estimator: pose estimation estimator
    """
    config = estimator.config

    dataset_test = SingleCubeDataset(
        config=config,
        split="test",
        zip_file_name=config.test.dataset_zip_file_name_test,
        data_root=config.system.data_root,
        sample_size=config.test.sample_size_test,
    )

    estimator.logger.info("Start evaluating estimator: %s", type(estimator).__name__)

    test_loader = torch.utils.data.DataLoader(
        dataset_test,
        batch_size=config.test.batch_test_size,
        num_workers=0,
        drop_last=False,
    )

    estimator.model.to(estimator.device)
    evaluate_one_epoch(
        estimator=estimator,
        config=config,
        data_loader=test_loader,
        epoch=0,
        test=True,
    )


def evaluate_one_epoch(*, estimator, config, data_loader, epoch, test):
    """Evaluation of the model on one epoch
    Args:
        estimator: pose estimation estimator
        config: configuration of the model
        data_loader (DataLoader): pytorch dataloader
        epoch (int): the current epoch number
        test (bool): specifies which type of evaluation we are doing
    """
    estimator.model.eval()
    estimator.logger.info(f" evaluation started")

    metric_translation = 0.0
    metric_scaleY = 0.0

    if test:
        batch_size = config.test.batch_test_size
    elif test == False:
        batch_size = config.val.batch_validation_size
    else:
        raise ValueError(f"You need to specify a boolean value for the test argument")

    number_batches = len(data_loader) / batch_size
    with torch.no_grad():
        metric_translation, metric_scaleY = evaluation_over_batch(
            estimator=estimator,
            config=config,
            data_loader=data_loader,
            batch_size=batch_size,
            epoch=epoch,
            is_training=False,
        )

        estimator.writer.log_evaluation(
            evaluation_metric_translation=metric_translation,
            evaluation_metric_scaleY = metric_scaleY,
            epoch=epoch,
            test=test,
        )


# HELPER
def evaluation_over_batch(
    *,
    estimator,
    config,
    data_loader,
    batch_size,
    epoch,
    is_training=True,
    optimizer=None,
    criterion_translation=None,
    criterion_scaleY = None,
):
    """
    Do the training process for all the estimators (one for each class)

    Args:
        estimator: pose estimation estimator
        config: configuration of the model
        data_loader (DataLoader): pytorch dataloader
        batch_size (int): size of the batch
        epoch (int): the current epoch number
        is_training (bool): boolean to say if we are in a training process or not
        optimizer: optimizer of the model
        criterion_translation (torch.nn): criterion for the evaluation of the translation loss
        criterion_orientation torch.nn: criterion for the evaluation of the orientation loss
    """
    
    sample_size = config.train.sample_size_train if is_training else config.val.sample_size_val
    len_data_loader = sample_size if (sample_size > 0) else len(data_loader)

    metric_translation = 0
    metric_scaleY = 0

    for index, (imagesRGB, imagesDepth, target_translation_list, target_scaleY_list) in enumerate(
        data_loader
    ):
        imagesRGB = list(image.to(estimator.device) for image in imagesRGB)
        imagesDepth = list(image.to(estimator.device) for image in imagesDepth)

        loss_translation = 0
        loss_scaleY = 0

        if estimator.model.is_symetric == False:
            output_translation, output_scaleY = estimator.model(
                torch.stack(imagesRGB).reshape(
                    -1, 3, config.dataset.image_scale, config.dataset.image_scale
                ),
                torch.stack(imagesDepth).reshape(
                    -1, 1, config.dataset.image_scale, config.dataset.image_scale
                )
            )

            target_translation = target_translation_list.to(estimator.device)
            target_scaleY = target_scaleY_list.to(estimator.device)

            metric_translation += translation_average_mean_square_error(
                output_translation, target_translation
            )


            metric_scaleY += translation_average_mean_square_error(
                output_scaleY, target_scaleY
            )
            

            intermediate_mean_loss_translation = metric_translation / (index + 1)
            intermediate_mean_loss_scaleY = metric_scaleY / (index + 1)
            estimator.logger.info(
                f"intermediate mean translation loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_translation}"
            )
            estimator.logger.info(
                f"intermediate mean scaleY loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_scaleY}"
            )

            if is_training:
                loss_translation += criterion_translation(
                    output_translation, target_translation
                )
                loss_scaleY += criterion_scaleY(
                    output_scaleY, target_scaleY
                )

                train_loss = (
                    loss_translation + loss_scaleY
                )

        else:
            output_translation, output_scaleY  = estimator.model(
                torch.stack(imagesRGB).reshape(
                    -1, 3, config.dataset.image_scale, config.dataset.image_scale
                ),
                torch.stack(imagesDepth).reshape(
                    -1, 1, config.dataset.image_scale, config.dataset.image_scale
                )
            )

            target_translation = target_translation_list.to(estimator.device)
            target_scaleY = target_scaleY_list.to(estimator.device)

            metric_translation += translation_average_mean_square_error(
                output_translation, target_translation
            )

            metric_scaleY += translation_average_mean_square_error(
                output_scaleY, target_scaleY
            )
            

            intermediate_mean_loss_translation = metric_translation / (index + 1)
            intermediate_mean_loss_scaleY = metric_scaleY / (index + 1)
            estimator.logger.info(
                f"intermediate mean translation loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_translation}"
            )
            estimator.logger.info(
                f"intermediate mean scaleY loss after mini batch {index + 1} in epoch {epoch} is: {intermediate_mean_loss_scaleY}"
            )

            if is_training:
                loss_translation += criterion_translation(
                    output_translation, target_translation
                )
                loss_scaleY += criterion_scaleY(
                    output_scaleY, target_scaleY
                )

                train_loss = loss_translation + loss_scaleY

        if is_training:
            train_loss.backward()

            if (index + 1) % config.train.accumulation_steps == 0:
                optimizer.step()
                optimizer.zero_grad()

    metric_translation = metric_translation / len_data_loader
    metric_scaleY = metric_scaleY / len_data_loader

    return metric_translation, metric_scaleY