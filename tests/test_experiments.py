import sys
import glob

def test_single_agent():
    initial_models = set(glob.glob('**/best_model.zip', recursive=True))
    sys.path.append('experiments/learning/')
    from experiments.learning.singleagent import run as run_train
    run_train(steps=30, output_folder='tmp')

    from experiments.learning.test_singleagent import run as run_test
    new_models = set(glob.glob('**/best_model.zip', recursive=True))
    test_model = new_models - initial_models
    assert len(test_model) > 0, initial_models
    path = '/'.join(next(iter(test_model)).split('/')[:-1])
    run_test(path, plot=False, gui=False, output_folder='tmp')

# def test_multi_agent(self):
#     initial_models = set(glob.glob('**/best_model.zip', recursive=True))
#     sys.path.append('experiments/learning')
#     from experiments.learning.multiagent import run as run_train
#     run_train(steps=30, output_folder='tmp')

#     from experiments.learning.test_multiagent import run as run_test
#     new_models = set(glob.glob('**/best_model.zip', recursive=True))
#     test_model = new_models - initial_models
#     assert len(test_model) > 0, initial_models
#     path = '/'.join(next(iter(test_model)).split('/')[:-1])
#     run_test(path, plot=False, gui=False, output_folder='tmp')
