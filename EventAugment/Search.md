# **Rand Search**

``` python
transforms = [
'Identity', 'AutoContrast', 'Equalize',
'Rotate', 'Solarize', 'Color', 'Posterize',
'Contrast', 'Brightness', 'Sharpness',
'ShearX', 'ShearY', 'TranslateX', 'TranslateY']
def randaugment(N, M):
    """
    Generate a set of distortions.
    Args:
    N: Number of augmentation transformations to apply sequentially.
    M: Magnitude for all the transformations.
    """
    sampled_ops = np.random.choice(transforms, N)
    return [(op, M) for op in sampled_ops]
```





