use crate::consts::TOTAL_PAGES;

pub struct PageMap {
    pub pages: [u8; TOTAL_PAGES],
    pub active_pages: usize,
    pub unlocked: bool,
    pub msp: usize,
    pub reset_vector: usize,
}

impl PageMap {
    pub fn get_page_mut(&mut self, page: usize) -> Option<&mut u8> {
        if !self.unlocked || page >= self.active_pages {
            return None;
        }

        self.pages.get_mut(page)
    }

    pub fn ready_to_write_vector_subpage(&self) -> bool {
        sprkt_log!(info, "map: {:?}", self.pages);

        if !(self.unlocked && self.active_pages > 0) {
            return false;
        }

        // TODO: I probably want to ensure that the vector table SUBPAGE is written first in this page, because
        // if we lose power, we could be in trouble.
        self.pages[0] == 0xFE
            && self
                .pages
                .iter()
                .skip(1)
                .take(self.active_pages - 1)
                .all(|p| *p == 0xFF)
    }

    pub fn bootload_complete(&self) -> bool {
        self.unlocked
            && self.active_pages > 0
            && self
                .pages
                .iter()
                .take(self.active_pages)
                .all(|p| *p == 0xFF)
    }
}
