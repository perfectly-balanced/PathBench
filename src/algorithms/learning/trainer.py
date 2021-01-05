from typing import TYPE_CHECKING, Dict, Any

if TYPE_CHECKING:
    from main import MainRunner


class Trainer:
    @staticmethod
    def main(m: 'MainRunner') -> None:
        config: Dict[str, Any] = m.main_services.settings.trainer_custom_config
        if not config:
            config = m.main_services.settings.trainer_model.get_config()
        # adding in-built map detection
        config['training_data'] = ['_'.join(['training'] + m.main_services.settings.generator_labelling_atlases)]

        model = m.main_services.settings.trainer_model(m.main_services, config).to(m.main_services.torch.device)
        model.full_train_holdout()
