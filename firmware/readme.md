To compile Arduino firmware, symlink this folder into your Arduino libraries folder.
This effectively creates a private library for the project.

```bash
ln -s /repo-path/firmware ~/Arduino/libraries/str1ker
```

Arduino adds the library directory itself to the include path, so includes
should reference the header name directly.

Use:

```c++
#include <firmware.h>
```
