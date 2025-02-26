//
//	Menu.cpp
//	========
//


#include "Environment.h"
#include "Menu.h"

//
//	All of the menus are defined here, statically.
//

const menu_memory menus PROGMEM = {{
		{{
			{{ 'C', 'a', 'b', ' ' },	ACTION_NEW_MOBILE	},
			{{ 'A', 'c', 'c', ' ' },	ACTION_NEW_STATIC	},
			{{ 'I', '/', 'O', ' ' },	ACTION_TOGGLE		},
			{{ ' ', ' ', ' ', 'v' },	ACTION_NEXT		}
		}},
		{{
			{{ 'S', 't', 'a', 't' },	ACTION_STATUS		},
			{{ 'D', 'e', 'l', ' ' },	ACTION_ERASE		},
			{{ ' ', ' ', ' ', ' ' },	ACTION_NONE		},
			{{ ' ', ' ', ' ', 'v' },	ACTION_NEXT		}
		}},
		{{
			{{ 'O', 'n', ' ', ' ' },	ACTION_START		},
			{{ 'O', 'f', 'f', ' ' },	ACTION_STOP		},
			{{ 'S', 'a', 'v', 'e' },	ACTION_SAVE		},
			{{ ' ', ' ', ' ', 'v' },	ACTION_NEXT		}
		}}
}};

//
//	EOF
//
