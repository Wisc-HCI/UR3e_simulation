import {isEqual} from 'lodash';

export const indexOf = (array,element) => {
    let index = -1;
    array.some((el,idx)=>{
        if (isEqual(el,element)) {
            index = idx;
            return true
        } else {
            return false
        }
    });
    return index;
}