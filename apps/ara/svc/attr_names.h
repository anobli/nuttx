#ifndef __SVC_ATTR_NAMES_H_
#define __SVC_ATTR_NAMES_H_

#include <stdint.h>

struct attr_name {
    uint16_t attr;
    const char *name;
};

struct attr_name_group {
    /*
     * This is an array, terminated with an element whose "name"
     * field is null.
     */
    const struct attr_name *attr_names;
    /*
     * A printable name of this group of attributes.
     */
    const char *group_name;
};

extern const struct attr_name_group unipro_l1_attr_group;
extern const struct attr_name_group unipro_l1_5_attr_group;
extern const struct attr_name_group unipro_l2_attr_group;
extern const struct attr_name_group unipro_l3_attr_group;
extern const struct attr_name_group unipro_l4_attr_group;
extern const struct attr_name_group unipro_dme_attr_group;

#endif
