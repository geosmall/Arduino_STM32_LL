#ifndef UTLIST_H_
#define UTLIST_H_

#define UTLIST_VERSION 1.8

/*
 * This file contains macros to manipulate singly and doubly-linked lists.
 *
 * 1. LL_ macros:  singly-linked lists.
 * 2. DL_ macros:  doubly-linked lists.
 * 3. CDL_ macros: circular doubly-linked lists.
 *
 * To use singly-linked lists, your structure must have a "next" pointer.
 * To use doubly-linked lists, your structure must "prev" and "next" pointers.
 * Either way, the pointer to the head of the list must be initialized to NULL.
 *
 * ----------------.EXAMPLE -------------------------
 * struct item {
 *      int id;
 *      struct item *prev, *next;
 * }
 *
 * struct item *list = NULL:
 *
 * int main() {
 *      struct item *item;
 *      ... allocate and populate item ...
 *      DL_APPEND(list, item);
 * }
 * --------------------------------------------------
 *
 * For doubly-linked lists, the append and delete macros are O(1)
 * For singly-linked lists, append and delete are O(n) but prepend is O(1)
 * The sort macro is O(n log(n)) for all types of single/double/circular lists.
 */

/******************************************************************************
 * The sort macro is an adaptation of Simon Tatham's O(n log(n)) mergesort    *
 * Unwieldy variable names used here to avoid shadowing passed-in variables.  *
 *****************************************************************************/
#define LL_SORT(list, cmp)                             \
    do {                                                                             \
        __typeof__(list)_ls_p, _ls_q, _ls_e, _ls_tail, _ls_oldhead;              \
        int _ls_insize, _ls_nmerges, _ls_psize, _ls_qsize, _ls_i, _ls_looping;         \
        if (list) {                                                                    \
            _ls_insize  = 1;                                                              \
            _ls_looping = 1;                                                             \
            while (_ls_looping) {                                                        \
                _ls_p       = list;                                                              \
                _ls_oldhead = list;                                                        \
                list        = NULL;                                                               \
                _ls_tail    = NULL;                                                           \
                _ls_nmerges = 0;                                                           \
                while (_ls_p) {                                                            \
                    _ls_nmerges++;                                                           \
                    _ls_q     = _ls_p;                                                           \
                    _ls_psize = 0;                                                           \
                    for (_ls_i = 0; _ls_i < _ls_insize; _ls_i++) {                           \
                        _ls_psize++;                                                           \
                        _ls_q = _ls_q->next;                                             \
                        if (!_ls_q) { break; }                                                     \
                    }                                                                        \
                    _ls_qsize = _ls_insize;                                                  \
                    while (_ls_psize > 0 || (_ls_qsize > 0 && _ls_q)) {                      \
                        if (_ls_psize == 0) {                                                  \
                            _ls_e = _ls_q; _ls_q = _ls_q->next; _ls_qsize--;                 \
                        } \
                        else if (_ls_qsize == 0 || !_ls_q) {                                 \
                            _ls_e = _ls_p; _ls_p = _ls_p->next; _ls_psize--;                 \
                        } \
                        else if (cmp(_ls_p, _ls_q) <= 0) {          \
                            _ls_e = _ls_p; _ls_p = _ls_p->next; _ls_psize--;                 \
                        } \
                        else {                                                               \
                            _ls_e = _ls_q; _ls_q = _ls_q->next; _ls_qsize--;                 \
                        }                                                                      \
                        if (_ls_tail) {                                                        \
                            _ls_tail->next = _ls_e;                                   \
                        } \
                        else {                                                               \
                            list = _ls_e;                                           \
                        }                                                                      \
                        _ls_tail = _ls_e;                                                      \
                    }                                                                        \
                    _ls_p = _ls_q;                                                           \
                }                                                                          \
                _ls_tail->next = NULL;                                               \
                if (_ls_nmerges <= 1) {                                                    \
                    _ls_looping = 0;                                                           \
                }                                                                          \
                _ls_insize *= 2;                                                           \
            }                                                                            \
        }                                                                              \
    } \
    while (0)

#define DL_SORT(list, cmp)                             \
    do {                                                                             \
        __typeof__(list)_ls_p, _ls_q, _ls_e, _ls_tail, _ls_oldhead;              \
        int _ls_insize, _ls_nmerges, _ls_psize, _ls_qsize, _ls_i, _ls_looping;         \
        if (list) {                                                                    \
            _ls_insize  = 1;                                                              \
            _ls_looping = 1;                                                             \
            while (_ls_looping) {                                                        \
                _ls_p       = list;                                                              \
                _ls_oldhead = list;                                                        \
                list        = NULL;                                                               \
                _ls_tail    = NULL;                                                           \
                _ls_nmerges = 0;                                                           \
                while (_ls_p) {                                                            \
                    _ls_nmerges++;                                                           \
                    _ls_q     = _ls_p;                                                           \
                    _ls_psize = 0;                                                           \
                    for (_ls_i = 0; _ls_i < _ls_insize; _ls_i++) {                           \
                        _ls_psize++;                                                           \
                        _ls_q = _ls_q->next;                                             \
                        if (!_ls_q) { break; }                                                     \
                    }                                                                        \
                    _ls_qsize = _ls_insize;                                                  \
                    while (_ls_psize > 0 || (_ls_qsize > 0 && _ls_q)) {                      \
                        if (_ls_psize == 0) {                                                  \
                            _ls_e = _ls_q; _ls_q = _ls_q->next; _ls_qsize--;                 \
                        } \
                        else if (_ls_qsize == 0 || !_ls_q) {                                 \
                            _ls_e = _ls_p; _ls_p = _ls_p->next; _ls_psize--;                 \
                        } \
                        else if (cmp(_ls_p, _ls_q) <= 0) {          \
                            _ls_e = _ls_p; _ls_p = _ls_p->next; _ls_psize--;                 \
                        } \
                        else {                                                               \
                            _ls_e = _ls_q; _ls_q = _ls_q->next; _ls_qsize--;                 \
                        }                                                                      \
                        if (_ls_tail) {                                                        \
                            _ls_tail->next = _ls_e;                                   \
                        } \
                        else {                                                               \
                            list = _ls_e;                                           \
                        }                                                                      \
                        _ls_e->prev = _ls_tail;                                   \
                        _ls_tail    = _ls_e;                                                      \
                    }                                                                        \
                    _ls_p = _ls_q;                                                           \
                }                                                                          \
                list->prev     = _ls_tail;                                      \
                _ls_tail->next = NULL;                                               \
                if (_ls_nmerges <= 1) {                                                    \
                    _ls_looping = 0;                                                           \
                }                                                                          \
                _ls_insize *= 2;                                                           \
            }                                                                            \
        }                                                                              \
    } \
    while (0)

#define CDL_SORT(list, cmp)                             \
    do {                                                                             \
        __typeof__(list)_ls_p, _ls_q, _ls_e, _ls_tail, _ls_oldhead;              \
        int _ls_insize, _ls_nmerges, _ls_psize, _ls_qsize, _ls_i, _ls_looping;         \
        if (list) {                                                                    \
            _ls_insize  = 1;                                                              \
            _ls_looping = 1;                                                             \
            while (_ls_looping) {                                                        \
                _ls_p       = list;                                                              \
                _ls_oldhead = list;                                                        \
                list        = NULL;                                                               \
                _ls_tail    = NULL;                                                           \
                _ls_nmerges = 0;                                                           \
                while (_ls_p) {                                                            \
                    _ls_nmerges++;                                                           \
                    _ls_q     = _ls_p;                                                           \
                    _ls_psize = 0;                                                           \
                    for (_ls_i = 0; _ls_i < _ls_insize; _ls_i++) {                           \
                        _ls_psize++;                                                           \
                        _ls_q = ((_ls_q->next == _ls_oldhead) ? NULL : _ls_q->next); \
                        if (!_ls_q) { break; }                                                     \
                    }                                                                        \
                    _ls_qsize = _ls_insize;                                                  \
                    while (_ls_psize > 0 || (_ls_qsize > 0 && _ls_q)) {                      \
                        if (_ls_psize == 0) {                                                  \
                            _ls_e = _ls_q; _ls_q = _ls_q->next; _ls_qsize--;                 \
                            if (_ls_q == _ls_oldhead) { _ls_q = NULL; }           \
                        } \
                        else if (_ls_qsize == 0 || !_ls_q) {                                 \
                            _ls_e = _ls_p; _ls_p = _ls_p->next; _ls_psize--;                 \
                            if (_ls_p == _ls_oldhead) { _ls_p = NULL; }         \
                        } \
                        else if (cmp(_ls_p, _ls_q) <= 0) {          \
                            _ls_e = _ls_p; _ls_p = _ls_p->next; _ls_psize--;                 \
                            if (_ls_p == _ls_oldhead) { _ls_p = NULL; }         \
                        } \
                        else {                                                               \
                            _ls_e = _ls_q; _ls_q = _ls_q->next; _ls_qsize--;                 \
                            if (_ls_q == _ls_oldhead) { _ls_q = NULL; }         \
                        }                                                                      \
                        if (_ls_tail) {                                                        \
                            _ls_tail->next = _ls_e;                                   \
                        } \
                        else {                                                               \
                            list = _ls_e;                                           \
                        }                                                                      \
                        _ls_e->prev = _ls_tail;                                   \
                        _ls_tail    = _ls_e;                                                      \
                    }                                                                        \
                    _ls_p = _ls_q;                                                           \
                }                                                                          \
                list->prev     = _ls_tail;                                      \
                _ls_tail->next = list;                                        \
                if (_ls_nmerges <= 1) {                                                    \
                    _ls_looping = 0;                                                           \
                }                                                                          \
                _ls_insize *= 2;                                                           \
            }                                                                            \
        }                                                                              \
    } \
    while (0)

/******************************************************************************
 * singly linked list macros (non-circular)                                   *
 *****************************************************************************/
#define LL_PREPEND(head, add)                                                     \
    do {                                                                             \
        (add)->next = head;                                                            \
        head = add;                                                                    \
    } \
    while (0)

#define LL_APPEND(head, add)                                                      \
    do {                                                                             \
        __typeof__(head)_tmp;                                                         \
        (add)->next = NULL;                                                              \
        if (head) {                                                                    \
            _tmp = head;                                                                 \
            while (_tmp->next) { _tmp = _tmp->next; }         \
            _tmp->next = (add);                                         \
        } \
        else {                                                                       \
            (head) = (add);                                                                \
        }                                                                              \
    } \
    while (0)

#define LL_DELETE(head, del)                                                      \
    do {                                                                             \
        __typeof__(head)_tmp;                                                         \
        if ((head) == (del)) {                                                         \
            (head) = (head)->next;                                                         \
        } \
        else {                                                                       \
            _tmp = head;                                                                 \
            while (_tmp->next && (_tmp->next != (del))) { \
                _tmp = _tmp->next;                                         \
            }                                                                            \
            if (_tmp->next) {                                                \
                _tmp->next = ((del)->next);                             \
            }                                                                            \
        }                                                                              \
    } \
    while (0)

#define LL_FOREACH(head, el)                                                      \
    for (el = head; el; el = el->next)

/******************************************************************************
 * doubly linked list macros (non-circular)                                   *
 *****************************************************************************/
#define DL_PREPEND(head, add)                                                     \
    do {                                                                             \
        (add)->next = head;                                                             \
        if (head) {                                                                     \
            (add)->prev  = (head)->prev;                                                   \
            (head)->prev = (add);                                                         \
        } \
        else {                                                                        \
            (add)->prev = (add);                                                          \
        }                                                                               \
        (head) = (add);                                                                 \
    } \
    while (0)

#define DL_APPEND(head, add)                                                      \
    do {                                                                             \
        if (head) {                                                                    \
            (add)->prev  = (head)->prev;                                                \
            (head)->prev->next = (add);                                                \
            (head)->prev = (add);                                                      \
            (add)->next  = NULL;                                                        \
        } \
        else {                                                                       \
            (head) = (add);                                                              \
            (head)->prev = (head);                                                     \
            (head)->next = NULL;                                                       \
        }                                                                              \
    } \
    while (0);

#define DL_DELETE(head, del)                                                      \
    do {                                                                             \
        if ((del)->prev == (del)) {                                                    \
            (head) = NULL;                                                               \
        } \
        else if ((del) == (head)) {                                                    \
            (del)->next->prev = (del)->prev;                                           \
            (head) = (del)->next;                                                      \
        } \
        else {                                                                       \
            (del)->prev->next = (del)->next;                                           \
            if ((del)->next) {                                                         \
                (del)->next->prev = (del)->prev;                                       \
            } \
            else {                                                                   \
                (head)->prev = (del)->prev;                                            \
            }                                                                          \
        }                                                                              \
    } \
    while (0);


#define DL_FOREACH(head, el)                                                      \
    for (el = head; el; el = el->next)

/******************************************************************************
 * circular doubly linked list macros                                         *
 *****************************************************************************/
#define CDL_PREPEND(head, add)                                                    \
    do {                                                                             \
        if (head) {                                                                     \
            (add)->prev  = (head)->prev;                                                   \
            (add)->next  = (head);                                                         \
            (head)->prev = (add);                                                         \
            (add)->prev->next = (add);                                                    \
        } \
        else {                                                                        \
            (add)->prev = (add);                                                          \
            (add)->next = (add);                                                          \
        }                                                                               \
        (head) = (add);                                                                    \
    } \
    while (0)

#define CDL_DELETE(head, del)                                                     \
    do {                                                                             \
        if (((head) == (del)) && ((head)->next == (head))) {                            \
            (head) = 0L;                                                               \
        } \
        else {                                                                       \
            (del)->next->prev = (del)->prev;                                            \
            (del)->prev->next = (del)->next;                                            \
            if ((del) == (head)) { (head) = (del)->next; }                                    \
        }                                                                              \
    } \
    while (0);

#define CDL_FOREACH(head, el)                                                     \
    for (el = head; el; el = (el->next == head ? 0L : el->next))

#endif // UTLIST_H_