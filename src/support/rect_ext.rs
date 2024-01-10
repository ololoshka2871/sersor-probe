use embedded_graphics::{geometry::Point, primitives::Line};

pub trait RectangleExt {
    fn top_right(&self) -> Option<Point>;
    fn bottom_left(&self) -> Option<Point>;

    fn diaganal1(&self) -> Line;
    fn diaganal2(&self) -> Line;

    fn left_line(&self) -> Line;
}

impl RectangleExt for embedded_graphics::primitives::Rectangle {
    fn top_right(&self) -> Option<Point> {
        if self.size.width > 0 && self.size.height > 0 {
            Some(
                Point::new(self.top_left.x + self.size.width as i32, self.top_left.y)
                    - Point::new(1, 1),
            )
        } else {
            None
        }
    }

    fn bottom_left(&self) -> Option<Point> {
        if self.size.width > 0 && self.size.height > 0 {
            Some(
                Point::new(self.top_left.x, self.top_left.y + self.size.height as i32)
                    - Point::new(1, 1),
            )
        } else {
            None
        }
    }

    fn diaganal1(&self) -> Line {
        Line::new(self.top_left, self.top_left + self.size - Point::new(1, 1))
    }

    fn diaganal2(&self) -> Line {
        let diag1 = self.diaganal1();
        Line::new(
            Point::new(diag1.start.x, diag1.end.y),
            Point::new(diag1.end.x, diag1.start.y),
        )
    }

    fn left_line(&self) -> Line {
        if self.size.height > 0 {
            Line::new(
                self.top_left,
                Point::new(
                    self.top_left.x,
                    self.top_left.y + self.size.height as i32 - 1,
                ),
            )
        } else {
            Line::new(self.top_left, self.top_left)
        }
    }
}
