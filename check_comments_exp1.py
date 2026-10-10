import os
import glob

files = glob.glob('Experiment-1-Beginner/RTL/*.*') + \
        glob.glob('Experiment-1-Beginner/Simulation/*.*') + \
        glob.glob('Experiment-1-Beginner/Testbench/*.*')

commented = []
uncommented = []

for filepath in files:
    if not (filepath.endswith('.v') or filepath.endswith('.py') or filepath.endswith('.sv')):
        continue
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
        
    has_module_comment = ('// Module:' in content) or ('# Module:' in content) or ('// ============================================================================' in content)
    
    if has_module_comment:
        commented.append(filepath)
    else:
        uncommented.append(filepath)

print("COMMENTED FILES:")
for f in commented:
    print(f" - {f}")
    
print("\nUNCOMMENTED FILES:")
for f in uncommented:
    print(f" - {f}")
