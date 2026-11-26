"""Optional configurable box proxy, explicitly not a calibrated ambulance."""
import math


def vehicle_config(config):
    if config.get('evidence_status')!='unvalidated_modelling_assumption':
        raise ValueError('This proxy has no physical ambulance validation')
    if not config.get('parameter_source'):raise ValueError('Parameter provenance required')
    for k in ['length','width','height','mass']:
        if type(config.get(k)) not in (float,int) or not math.isfinite(config[k]) or config[k]<=0:
            raise ValueError('Positive finite proxy geometry/mass required')
    return dict(vehicle_model='varying_dynamics_bounding_box',**{k:config[k] for k in ['length','width','height','mass']})
